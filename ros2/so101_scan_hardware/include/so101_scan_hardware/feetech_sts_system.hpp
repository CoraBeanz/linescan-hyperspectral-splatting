// ros2_control hardware interface for the SO-101's Feetech STS3215 servos.
//
// The controller manager calls read() and write() once per control cycle.
// read() gets position, speed, load, voltage and temperature from every servo
// in one sync read; write() sends every goal position in one sync write. The
// servos run their own position loops, so this only converts units:
//
//   joint angle [rad] = sign * (ticks - zero_ticks) * 2 pi / 4096
//
// with zero_ticks, sign and the tick limits per joint from the calibration
// file (see so101_scan_hardware/calibrate.py), passed in as URDF parameters.
//
// Activating switches the motors on holding the pose the arm is in. Deactivating
// and shutting down leave them holding (disable_torque_on_deactivate: false), so
// stopping the launch doesn't drop the arm; torque:=false switches them off.
//
// Safety, every cycle, before anything reaches the servos:
//   * servo limits (servo_checks.hpp): load, temperature, supply voltage, the servos' own
//     alarms, and stalls. Passing one stops the arm: it holds where it is (the goal becomes
//     the measured pose, so a joint pushing on something stops pushing) and ignores the
//     controllers. If the cause is still there torque_off_after seconds later, the motors go
//     off, since holding isn't helping.
//   * soft limits (arm_model.hpp): a goal that would take the head or arm out of the
//     workspace, or past the gravity torque budget, is not sent; the arm stops at the edge
//     and can still move back in.
//   * e-stop (arm_safety_node.hpp): /arm_safety/estop holds, /arm_safety/torque_off goes
//     limp, true on /estop holds. /arm_safety/reset clears a stop once its cause is gone;
//     the arm then holds until a controller asks for the pose it is in, so a stale command
//     can't make it jump.
// None of this replaces the power switch: cutting the servo supply is the real e-stop.
#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "so101_scan_hardware/arm_model.hpp"
#include "so101_scan_hardware/arm_safety_node.hpp"
#include "so101_scan_hardware/servo_checks.hpp"
#include "so101_scan_hardware/sts_bus.hpp"

namespace so101_scan_hardware
{

class FeetechStsSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FeetechStsSystem)

  ~FeetechStsSystem() override;

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  struct Joint
  {
    std::string name;
    uint8_t id = 0;
    double zero_ticks = 2048.0;
    int sign = 1;
    int min_ticks = 0;
    int max_ticks = 4095;

    // state
    int ticks = 0;             // Present_Position as read
    double position = 0.0;     // rad
    double velocity = 0.0;     // rad/s
    double load = 0.0;         // fraction of the servo's maximum torque, signed
    double voltage = 0.0;      // V
    double temperature = 0.0;  // deg C
    // command from the controller, and what was last sent after rate limiting
    double command = 0.0;
    double sent = 0.0;
    int missed = 0;            // consecutive reads without an answer
  };

  // Unit conversions, public so the tests can check them. rad_to_ticks keeps to the
  // register's 0..4095; limits() is the calibrated range in rad, lower end first.
  static double ticks_to_rad(const Joint & j, double ticks);
  static int rad_to_ticks(const Joint & j, double rad);
  static std::pair<double, double> limits(const Joint & j);

  // How far outside its calibrated range a joint may be for activation to go ahead (~5 deg).
  static constexpr int kRangeSlackTicks = 57;

  // The e-stop and reset, as the safety services call them (public for the tests).
  // stop: hold the arm where it is, or switch the motors off. Returns whether it is stopped.
  std::pair<bool, std::string> stop(bool torque_off, const std::string & reason);
  // reset: let the arm move again once the cause of the stop is gone.
  std::pair<bool, std::string> reset();
  SafetyStatus safety_status();

private:
  bool read_all(bool update_commands);
  bool configure_servo(const Joint & j);
  void set_torque(bool on);
  std::vector<uint8_t> ids() const;

  // with bus_mutex_ held
  std::pair<bool, std::string> stop_locked(bool torque_off, const std::string & reason);
  void check_servos(double dt);
  std::vector<double> sent() const;
  // why a step from `from` to `to` breaks a soft limit, or "" if it doesn't
  std::string soft_limit(const ArmModel::Evaluation & to, const ArmModel::Evaluation & from) const;
  std::string outside_range() const;
  void update_status();
  void stop_safety_node();

  rclcpp::Logger logger_ = rclcpp::get_logger("FeetechStsSystem");
  StsBus bus_;
  // read() and write() run on the control loop's thread and the lifecycle callbacks on the
  // controller manager's executor, so a state change by hand could otherwise share the bus
  std::mutex bus_mutex_;
  std::vector<Joint> joints_;

  std::string port_;
  int baud_rate_ = 1000000;
  int timeout_ms_ = 10;
  bool torque_ = true;
  bool disable_torque_on_deactivate_ = false;
  int acceleration_ = 254;
  double max_velocity_ = 2.0;
  int max_missed_reads_ = 10;
  int return_delay_time_ = -1;
  int p_coefficient_ = -1;
  int d_coefficient_ = -1;
  int i_coefficient_ = -1;
  bool torque_enabled_ = false;

  // safety
  ServoChecks checks_;
  ArmModel model_;
  WorkspaceLimits workspace_;
  bool soft_limits_ = true;
  double stall_torque_ = 1.62;       // N m, STS3215 at 6 V
  double warn_gravity_load_ = 0.5;   // fractions of stall_torque_
  double max_gravity_load_ = 0.6;
  double torque_off_after_ = 5.0;    // s; 0 never switches the motors off on its own
  double resume_tolerance_ = 0.05;   // rad
  bool safety_node_enabled_ = true;
  SafetyState state_ = SafetyState::kInactive;
  std::string reason_;
  double fault_time_ = 0.0;          // s the cause of a stop has still been there while holding
  std::vector<std::string> warnings_, present_;
  std::string soft_limit_;           // what the soft limits are holding back, if anything
  std::mutex status_mutex_;          // guards status_, which the safety node reads
  SafetyStatus status_;
  std::unique_ptr<ArmSafetyNode> safety_node_;
};

}  // namespace so101_scan_hardware
