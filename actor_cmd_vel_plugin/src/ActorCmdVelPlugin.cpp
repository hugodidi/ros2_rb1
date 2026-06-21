#include <geometry_msgs/msg/twist.hpp>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/msgs/pose.pb.h>
#include <ignition/transport/Node.hh>
#include <mutex>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <thread>

using namespace ignition;
using namespace gazebo;

class ActorCmdVelPlugin : public gz::sim::System,
                          public gz::sim::ISystemConfigure,
                          public gz::sim::ISystemPreUpdate {
public:
  ActorCmdVelPlugin()
      : node(nullptr), actorEntity(gz::sim::kNullEntity) {}

  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm, EventManager &) override {

    this->actorEntity = _entity;

    // Leer parámetros del SDF.
    this->cmdVelTopic = "/actor/cmd_vel";
    this->setPoseTopic = "/world/LeonHome/set_pose";
    this->maxLinearVel = 1.0;
    this->maxAngularVel = 1.0;
    this->rotationCenterOffset = ignition::math::Vector3d::Zero;
    // Offset que alinea el eje longitudinal del teleop (linear.x) con la
    // dirección visual de avance del actor. En el modelo de Fuel el personaje
    // mira hacia el eje +Y del actor, mientras que Gazebo toma yaw=0 como +X.
    this->headingOffset = IGN_PI / 2.0;

    if (_sdf) {
      if (_sdf->HasElement("cmd_vel_topic")) {
        this->cmdVelTopic = _sdf->Get<std::string>("cmd_vel_topic");
      }
      if (_sdf->HasElement("set_pose_topic")) {
        this->setPoseTopic = _sdf->Get<std::string>("set_pose_topic");
      }
      if (_sdf->HasElement("max_linear_vel")) {
        this->maxLinearVel = _sdf->Get<double>("max_linear_vel");
      }
      if (_sdf->HasElement("max_angular_vel")) {
        this->maxAngularVel = _sdf->Get<double>("max_angular_vel");
      }
      if (_sdf->HasElement("rotation_center_offset")) {
        this->rotationCenterOffset = _sdf->Get<ignition::math::Vector3d>(
            "rotation_center_offset");
      }
      if (_sdf->HasElement("heading_offset")) {
        this->headingOffset = _sdf->Get<double>("heading_offset");
      }
    }

    auto nameComp = _ecm.Component<components::Name>(this->actorEntity);
    if (nameComp) {
      this->actorName = nameComp->Data();
    }
    this->posePub = this->gzNode.Advertise<ignition::msgs::Pose>(
        this->setPoseTopic);

    rclcpp::init(0, nullptr);
    this->node = std::make_shared<rclcpp::Node>("actor_cmd_vel_node");

    this->cmdVelSub =
        this->node->create_subscription<geometry_msgs::msg::Twist>(
            this->cmdVelTopic, 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
              std::lock_guard<std::mutex> lock(this->mutex);
              this->linearVel = ignition::math::Vector3d(
                  msg->linear.x, msg->linear.y, msg->linear.z);
              this->angularVel = ignition::math::Vector3d(
                  msg->angular.x, msg->angular.y, msg->angular.z);
              RCLCPP_INFO(this->node->get_logger(),
                          "Cmd Vel Received: linear=(%.2f, %.2f, %.2f) "
                          "angular=(%.2f, %.2f, %.2f)",
                          this->linearVel.X(), this->linearVel.Y(),
                          this->linearVel.Z(), this->angularVel.X(),
                          this->angularVel.Y(), this->angularVel.Z());
            });

    this->executor =
        std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    this->executor->add_node(this->node);
    this->rosThread = std::thread([this]() {
      rclcpp::Rate rate(100); // Hz
      while (rclcpp::ok()) {
        this->executor->spin_some();
        rate.sleep();
      }
    });

    RCLCPP_INFO(this->node->get_logger(),
                "Plugin configurado. actor=%s topic=%s set_pose=%s "
                "max_linear=%.2f max_angular=%.2f heading_offset=%.3f "
                "rotation_center_offset=(%.3f, %.3f, %.3f)",
                this->actorName.c_str(),
                this->cmdVelTopic.c_str(), this->setPoseTopic.c_str(),
                this->maxLinearVel, this->maxAngularVel,
                this->headingOffset,
                this->rotationCenterOffset.X(),
                this->rotationCenterOffset.Y(),
                this->rotationCenterOffset.Z());
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override {
    std::lock_guard<std::mutex> lock(this->mutex);

    if (this->actorEntity == gz::sim::kNullEntity) {
      RCLCPP_WARN(this->node->get_logger(), "Entidad del actor no válida.");
      return;
    }

    auto poseComp = _ecm.Component<components::Pose>(this->actorEntity);
    if (!poseComp) {
      return;
    }

    auto currentPose = poseComp->Data();
    double dt =
        std::chrono::duration<double>(_info.simTime - this->lastSimTime)
            .count();
    this->lastSimTime = _info.simTime;

    if (dt <= 0.0) {
      return;
    }

    // Limitar velocidades.
    double v = ignition::math::clamp(this->linearVel.X(),
                                     -this->maxLinearVel, this->maxLinearVel);
    double vy = ignition::math::clamp(this->linearVel.Y(),
                                      -this->maxLinearVel, this->maxLinearVel);
    double w = ignition::math::clamp(this->angularVel.Z(),
                                     -this->maxAngularVel,
                                     this->maxAngularVel);

    double yaw = currentPose.Rot().Yaw();
    double dYaw = w * dt;
    double newYaw = yaw + dYaw;

    // Centro de rotación en coordenadas del mundo. Si el modelo visual no está
    // centrado en el origen del actor, este offset permite girar alrededor del
    // centro geométrico visible.
    ignition::math::Vector3d centerOffsetWorld =
        currentPose.Rot() * this->rotationCenterOffset;
    ignition::math::Vector3d rotationCenter =
        currentPose.Pos() + centerOffsetWorld;

    // Girar el origen del actor alrededor del centro de rotación.
    ignition::math::Quaterniond qYawDelta(0, 0, dYaw);
    ignition::math::Vector3d rotatedPos =
        rotationCenter +
        qYawDelta * (currentPose.Pos() - rotationCenter);

    // Avanzar en la dirección visual de avance del actor. El heading_offset
    // alinea linear.x (teleop "adelante") con el eje longitudinal del modelo.
    // linear.y actúa como strafe lateral.
    double motionYaw = newYaw + this->headingOffset;
    double dx = dt * (v * std::cos(motionYaw) - vy * std::sin(motionYaw));
    double dy = dt * (v * std::sin(motionYaw) + vy * std::cos(motionYaw));

    ignition::math::Vector3d newPos(
        rotatedPos.X() + dx, rotatedPos.Y() + dy, rotatedPos.Z());

    ignition::math::Pose3d newPose(
        newPos, ignition::math::Quaterniond(0, 0, newYaw));

    _ecm.SetComponentData<components::Pose>(this->actorEntity, newPose);
    this->PublishPoseToGazebo(newPose);
  }

  ~ActorCmdVelPlugin() {
    if (this->executor) {
      this->executor->cancel();
    }
    rclcpp::shutdown();
    if (this->rosThread.joinable()) {
      this->rosThread.join();
    }
  }

private:
  void PublishPoseToGazebo(const ignition::math::Pose3d &pose) {
    ignition::msgs::Pose msg;
    msg.set_name(this->actorName);
    msg.mutable_position()->set_x(pose.Pos().X());
    msg.mutable_position()->set_y(pose.Pos().Y());
    msg.mutable_position()->set_z(pose.Pos().Z());
    msg.mutable_orientation()->set_x(pose.Rot().X());
    msg.mutable_orientation()->set_y(pose.Rot().Y());
    msg.mutable_orientation()->set_z(pose.Rot().Z());
    msg.mutable_orientation()->set_w(pose.Rot().W());
    this->posePub.Publish(msg);
  }

  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub;
  ignition::math::Vector3d linearVel{0, 0, 0};
  ignition::math::Vector3d angularVel{0, 0, 0};
  std::mutex mutex;
  std::chrono::steady_clock::duration lastSimTime{};
  gz::sim::Entity actorEntity;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::thread rosThread;

  std::string cmdVelTopic{"/actor/cmd_vel"};
  std::string setPoseTopic{"/world/LeonHome/set_pose"};
  std::string actorName{"actor_gesture"};
  ignition::transport::Node gzNode;
  ignition::transport::Node::Publisher posePub;
  double maxLinearVel{1.0};
  double maxAngularVel{1.0};
  double headingOffset{0.0};
  ignition::math::Vector3d rotationCenterOffset{0, 0, 0};
};

IGNITION_ADD_PLUGIN(ActorCmdVelPlugin, gz::sim::System,
                    gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
