// The driver's ROS side of the arm safety: e-stop and reset services, the /estop topic, and the
// safety state on /arm_safety/state (so101_scan_interfaces/ArmSafety).
//
// It is a small node of its own with its own thread, inside ros2_control_node, so an e-stop
// reaches the driver whatever the controllers are doing, and the control loop never waits on
// ROS: the services call into the driver, which takes the bus for the few milliseconds a stop
// needs, and the state is read from a copy the driver refreshes every cycle.
#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "so101_scan_interfaces/msg/arm_safety.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace so101_scan_hardware
{

// The driver's safety state, as published (the numbers are ArmSafety's constants).
enum class SafetyState : uint8_t
{
  kOk = 0,
  kResuming = 1,
  kHolding = 2,
  kTorqueOff = 3,
  kReadOnly = 4,
  kInactive = 5,
};

struct SafetyStatus
{
  SafetyState state = SafetyState::kInactive;
  std::string reason;
  std::vector<std::string> warnings;
  std::vector<std::string> joint_names;
  std::vector<double> load, temperature, voltage, tracking_error, gravity_load;
  double workspace_margin = 0.0;
};

const char * to_string(SafetyState state);

class ArmSafetyNode
{
public:
  struct Callbacks
  {
    // Stops the arm (holding it, or with the motors off); returns whether it did, and a message.
    std::function<std::pair<bool, std::string>(bool torque_off, const std::string & reason)> stop;
    std::function<std::pair<bool, std::string>()> reset;
    std::function<SafetyStatus()> status;
  };

  // name: the node's name, so the services are /<name>/estop, /<name>/torque_off, /<name>/reset
  // and the state /<name>/state. estop_topic: a std_msgs/Bool topic where true stops the arm.
  ArmSafetyNode(Callbacks callbacks, const std::string & name = "arm_safety",
    const std::string & estop_topic = "/estop");
  ~ArmSafetyNode();
  ArmSafetyNode(const ArmSafetyNode &) = delete;
  ArmSafetyNode & operator=(const ArmSafetyNode &) = delete;

private:
  void tick();

  Callbacks callbacks_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<so101_scan_interfaces::msg::ArmSafety>::SharedPtr state_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> services_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread thread_;

  SafetyState last_state_ = SafetyState::kInactive;
  std::string last_reason_;
  std::vector<std::string> last_warnings_;
  rclcpp::Time last_published_;
};

}  // namespace so101_scan_hardware
