#include "so101_scan_hardware/arm_safety_node.hpp"

#include <chrono>

namespace so101_scan_hardware
{

using so101_scan_interfaces::msg::ArmSafety;
using std_srvs::srv::Trigger;
using namespace std::chrono_literals;

static_assert(static_cast<uint8_t>(SafetyState::kOk) == ArmSafety::OK, "states match ArmSafety.msg");
static_assert(static_cast<uint8_t>(SafetyState::kResuming) == ArmSafety::RESUMING, "states match ArmSafety.msg");
static_assert(static_cast<uint8_t>(SafetyState::kHolding) == ArmSafety::HOLDING, "states match ArmSafety.msg");
static_assert(static_cast<uint8_t>(SafetyState::kTorqueOff) == ArmSafety::TORQUE_OFF, "states match ArmSafety.msg");
static_assert(static_cast<uint8_t>(SafetyState::kReadOnly) == ArmSafety::READ_ONLY, "states match ArmSafety.msg");
static_assert(static_cast<uint8_t>(SafetyState::kInactive) == ArmSafety::INACTIVE, "states match ArmSafety.msg");

const char * to_string(SafetyState state)
{
  switch (state) {
    case SafetyState::kOk: return "ok";
    case SafetyState::kResuming: return "resuming";
    case SafetyState::kHolding: return "holding";
    case SafetyState::kTorqueOff: return "torque off";
    case SafetyState::kReadOnly: return "read only";
    case SafetyState::kInactive: return "inactive";
  }
  return "?";
}

ArmSafetyNode::ArmSafetyNode(Callbacks callbacks, const std::string & name, const std::string & estop_topic)
: callbacks_(std::move(callbacks))
{
  // Its own arguments only: ros2_control_node's remappings and parameters are the controller
  // manager's, and a node name remapping there would rename this node too.
  node_ = std::make_shared<rclcpp::Node>(name, rclcpp::NodeOptions().use_global_arguments(false)
      .start_parameter_services(false).start_parameter_event_publisher(false));
  state_pub_ = node_->create_publisher<ArmSafety>("~/state", rclcpp::QoS(1).reliable().transient_local());

  auto trigger = [this](const char * service, auto handler) {
      services_.push_back(node_->create_service<Trigger>(service,
        [handler](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response) {
          std::tie(response->success, response->message) = handler();
        }));
    };
  trigger("~/estop", [this]() {return callbacks_.stop(false, "e-stop (" + std::string(node_->get_fully_qualified_name()) +
             "/estop)");});
  trigger("~/torque_off", [this]() {return callbacks_.stop(true, "e-stop with the motors off (" +
             std::string(node_->get_fully_qualified_name()) + "/torque_off)");});
  trigger("~/reset", [this]() {return callbacks_.reset();});
  estop_sub_ = node_->create_subscription<std_msgs::msg::Bool>(estop_topic, rclcpp::QoS(10).reliable(),
      [this, estop_topic](const std_msgs::msg::Bool & msg) {
        if (msg.data) {
          callbacks_.stop(false, "e-stop (" + estop_topic + ")");
        }
      });
  // Checked at 50 Hz, published on every change and otherwise at 10 Hz.
  timer_ = node_->create_wall_timer(20ms, [this]() {tick();});
  last_published_ = node_->now();

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  thread_ = std::thread([this]() {executor_->spin();});
}

ArmSafetyNode::~ArmSafetyNode()
{
  executor_->cancel();
  if (thread_.joinable()) {
    thread_.join();
  }
  executor_->remove_node(node_);
}

void ArmSafetyNode::tick()
{
  const SafetyStatus s = callbacks_.status();
  const auto now = node_->now();
  // the warnings carry live numbers, so only a new or cleared one counts as a change
  const bool changed = s.state != last_state_ || s.reason != last_reason_ ||
    s.warnings.size() != last_warnings_.size();
  if (!changed && (now - last_published_).seconds() < 0.1) {
    return;
  }
  ArmSafety msg;
  msg.header.stamp = now;
  msg.state = static_cast<uint8_t>(s.state);
  msg.reason = s.reason;
  msg.warnings = s.warnings;
  msg.joint_names = s.joint_names;
  msg.load = s.load;
  msg.temperature = s.temperature;
  msg.voltage = s.voltage;
  msg.tracking_error = s.tracking_error;
  msg.gravity_load = s.gravity_load;
  msg.workspace_margin = s.workspace_margin;
  state_pub_->publish(msg);
  last_state_ = s.state;
  last_reason_ = s.reason;
  last_warnings_ = s.warnings;
  last_published_ = now;
}

}  // namespace so101_scan_hardware
