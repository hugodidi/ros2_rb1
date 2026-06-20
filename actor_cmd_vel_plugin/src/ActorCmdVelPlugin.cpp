#include <geometry_msgs/msg/twist.hpp>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/math/Vector3.hh>
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
  ActorCmdVelPlugin() : node(nullptr), actorEntity(gz::sim::kNullEntity) {}

  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &,
                 EntityComponentManager &_ecm, EventManager &) override {

    this->actorEntity = _entity;
    rclcpp::init(0, nullptr);
    this->node = std::make_shared<rclcpp::Node>("actor_cmd_vel_node");

    this->cmdVelSub =
        this->node->create_subscription<geometry_msgs::msg::Twist>(
            "/actor/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
              std::lock_guard<std::mutex> lock(this->mutex);
              this->linearVel = ignition::math::Vector3d(
                  msg->linear.x, msg->linear.y, msg->linear.z);
              RCLCPP_INFO(this->node->get_logger(),
                          "Cmd Vel Received: x=%.2f, y=%.2f, z=%.2f",
                          linearVel.X(), linearVel.Y(), linearVel.Z());
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
                "Plugin configurado y esperando cmd_vel...");
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
    double dt = std::chrono::duration<double>(_info.simTime - this->lastSimTime)
                    .count();
    this->lastSimTime = _info.simTime;

    ignition::math::Vector3d delta = this->linearVel * dt;
    ignition::math::Vector3d newPos = currentPose.Pos() + delta;

    double yaw = currentPose.Rot().Yaw();

    ignition::math::Pose3d newPose(newPos,
                                   ignition::math::Quaterniond(0, 0, yaw));

    _ecm.SetComponentData<components::Pose>(this->actorEntity, newPose);
  }

  ~ActorCmdVelPlugin() { rclcpp::shutdown(); }

private:
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub;
  ignition::math::Vector3d linearVel{0, 0, 0};
  std::mutex mutex;
  std::chrono::steady_clock::duration lastSimTime{};
  gz::sim::Entity actorEntity;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::thread rosThread;
};

IGNITION_ADD_PLUGIN(ActorCmdVelPlugin, gz::sim::System,
                    gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
