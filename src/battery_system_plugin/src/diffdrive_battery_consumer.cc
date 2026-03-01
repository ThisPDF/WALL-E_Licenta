#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/JointVelocity.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

namespace gz::sim::systems
{
class DiffDriveBatteryConsumer
    : public System,
      public ISystemConfigure,
      public ISystemPreUpdate,
      public ISystemPostUpdate
{
  // ───────────────────────── Configure ─────────────────────────
  public: void Configure(
      const Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      EntityComponentManager &/*_ecm*/,
      EventManager &) override
  {
    this->modelEntity = _entity;

    if (_sdf)
    {
      if (_sdf->HasElement("left_joint"))
        this->leftJointName = _sdf->Get<std::string>("left_joint");
      if (_sdf->HasElement("right_joint"))
        this->rightJointName = _sdf->Get<std::string>("right_joint");
      if (_sdf->HasElement("capacity"))
        this->capacityAh = std::max(0.01, _sdf->Get<double>("capacity"));
      if (_sdf->HasElement("initial_charge"))
        this->chargeAh = std::clamp(
            _sdf->Get<double>("initial_charge"), 0.0, this->capacityAh);
      else
        this->chargeAh = this->capacityAh;
      if (_sdf->HasElement("voltage"))
        this->voltage = std::max(0.1, _sdf->Get<double>("voltage"));
      if (_sdf->HasElement("house_load_w"))
        this->houseLoadW = std::max(0.0, _sdf->Get<double>("house_load_w"));
      if (_sdf->HasElement("efficiency"))
        this->efficiency = std::clamp(
            _sdf->Get<double>("efficiency"), 0.05, 1.0);
      if (_sdf->HasElement("idle_per_motor_w"))
        this->idlePerMotorW = std::max(0.0, _sdf->Get<double>("idle_per_motor_w"));
      if (_sdf->HasElement("vel_coeff_w_per_radps"))
        this->velCoeff = std::max(0.0, _sdf->Get<double>("vel_coeff_w_per_radps"));
      if (_sdf->HasElement("acc_coeff_w_per_radps2"))
        this->accCoeff = std::max(0.0, _sdf->Get<double>("acc_coeff_w_per_radps2"));
      if (_sdf->HasElement("publish_rate_hz"))
        this->publishRateHz = std::max(0.1, _sdf->Get<double>("publish_rate_hz"));
      if (_sdf->HasElement("ros_topic"))
        this->rosTopic = _sdf->Get<std::string>("ros_topic");
    }

    // ── Initialise ROS2 (safe even if already initialised) ──
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
    this->rosNode = std::make_shared<rclcpp::Node>(
        "diffdrive_battery_consumer");
    this->batteryPctPub = this->rosNode->create_publisher<std_msgs::msg::Float32>(
        this->rosTopic, rclcpp::QoS(10));

    this->configured = true;
  }

  // ───────────────────────── PreUpdate ─────────────────────────
  // Mutable ECM → we can resolve joints & create velocity components here.
  public: void PreUpdate(
      const UpdateInfo &/*_info*/,
      EntityComponentManager &_ecm) override
  {
    if (!this->configured)
      return;

    // Lazy-resolve model (might not exist in Configure yet)
    if (!this->model.Valid(_ecm))
    {
      this->model = Model(this->modelEntity);
      if (!this->model.Valid(_ecm))
        return;
    }

    // Lazy-resolve joints (spawned models aren't complete during Configure)
    if (this->leftJoint == kNullEntity)
    {
      this->leftJoint = this->model.JointByName(_ecm, this->leftJointName);
    }
    if (this->rightJoint == kNullEntity)
    {
      this->rightJoint = this->model.JointByName(_ecm, this->rightJointName);
    }

    // Create JointVelocity components so Gazebo physics populates them
    this->EnsureJointVelocityComponent(_ecm, this->leftJoint);
    this->EnsureJointVelocityComponent(_ecm, this->rightJoint);
  }

  // ──────────────────────── PostUpdate ─────────────────────────
  // Const ECM → only read joint velocities, compute drain, publish.
  public: void PostUpdate(
      const UpdateInfo &_info,
      const EntityComponentManager &_ecm) override
  {
    if (!this->configured || _info.paused)
      return;

    const double dt = std::chrono::duration<double>(_info.dt).count();
    if (dt <= 0.0)
      return;

    // ── Read wheel velocities ──
    const double leftOmega  = this->ReadJointVelocity(_ecm, this->leftJoint);
    const double rightOmega = this->ReadJointVelocity(_ecm, this->rightJoint);

    const double leftAlpha  = (leftOmega  - this->prevLeftOmega)  / dt;
    const double rightAlpha = (rightOmega - this->prevRightOmega) / dt;
    this->prevLeftOmega  = leftOmega;
    this->prevRightOmega = rightOmega;

    // ── Power model ──
    const double wheelPowerW =
        (2.0 * this->idlePerMotorW
         + this->velCoeff * (std::abs(leftOmega)  + std::abs(rightOmega))
         + this->accCoeff * (std::abs(leftAlpha)   + std::abs(rightAlpha)))
        / this->efficiency;

    const double totalPowerW = this->houseLoadW + wheelPowerW;
    const double consumedAh  = (totalPowerW * dt) / (this->voltage * 3600.0);
    this->chargeAh = std::clamp(this->chargeAh - consumedAh, 0.0, this->capacityAh);

    // ── Throttled ROS2 publish ──
    const auto now = _info.simTime;
    const double sinceLastPub =
        std::chrono::duration<double>(now - this->lastPublishTime).count();

    if (this->lastPublishTime == std::chrono::steady_clock::duration::zero()
        || sinceLastPub >= (1.0 / this->publishRateHz))
    {
      std_msgs::msg::Float32 msg;
      msg.data = static_cast<float>(
          (this->chargeAh / this->capacityAh) * 100.0);

      if (this->batteryPctPub)
      {
        this->batteryPctPub->publish(msg);
        // Flush the publisher so the message actually goes out
        rclcpp::spin_some(this->rosNode);
      }
      this->lastPublishTime = now;
    }
  }

  // ───────────────────────── Helpers ──────────────────────────
  private: void EnsureJointVelocityComponent(
      EntityComponentManager &_ecm, Entity _joint)
  {
    if (_joint == kNullEntity)
      return;
    if (!_ecm.Component<components::JointVelocity>(_joint))
    {
      _ecm.CreateComponent(_joint,
          components::JointVelocity({0.0}));
    }
  }

  private: double ReadJointVelocity(
      const EntityComponentManager &_ecm, Entity _joint) const
  {
    if (_joint == kNullEntity)
      return 0.0;
    const auto *comp = _ecm.Component<components::JointVelocity>(_joint);
    if (!comp || comp->Data().empty())
      return 0.0;
    return comp->Data().front();
  }

  // ───────────────────────── Members ──────────────────────────
  private: bool configured{false};
  private: Entity modelEntity{kNullEntity};
  private: Model model{kNullEntity};
  private: Entity leftJoint{kNullEntity};
  private: Entity rightJoint{kNullEntity};

  private: std::string leftJointName{"left_back_wheel"};
  private: std::string rightJointName{"right_back_wheel"};
  private: std::string rosTopic{"/battery/percentage"};

  private: double capacityAh{5.6};
  private: double chargeAh{5.6};
  private: double voltage{11.1};
  private: double houseLoadW{0.0};
  private: double efficiency{0.75};
  private: double idlePerMotorW{3.0};
  private: double velCoeff{0.8};
  private: double accCoeff{0.2};
  private: double publishRateHz{2.0};

  private: double prevLeftOmega{0.0};
  private: double prevRightOmega{0.0};
  private: std::chrono::steady_clock::duration lastPublishTime{
      std::chrono::steady_clock::duration::zero()};

  private: rclcpp::Node::SharedPtr rosNode;
  private: rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr batteryPctPub;
};

GZ_ADD_PLUGIN(
    DiffDriveBatteryConsumer,
    System,
    DiffDriveBatteryConsumer::ISystemConfigure,
    DiffDriveBatteryConsumer::ISystemPreUpdate,
    DiffDriveBatteryConsumer::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(
    DiffDriveBatteryConsumer,
    "gz::sim::systems::DiffDriveBatteryConsumer")
}