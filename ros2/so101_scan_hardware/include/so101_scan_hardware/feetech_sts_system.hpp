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
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "so101_scan_hardware/sts_bus.hpp"

namespace so101_scan_hardware
{

class FeetechStsSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FeetechStsSystem)

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

  // Unit conversions, public so the tests can check them.
  static double ticks_to_rad(const Joint & j, double ticks);
  static int rad_to_ticks(const Joint & j, double rad);

private:
  bool read_all(bool update_commands);
  bool configure_servo(const Joint & j);
  void set_torque(bool on);
  std::vector<uint8_t> ids() const;

  rclcpp::Logger logger_ = rclcpp::get_logger("FeetechStsSystem");
  StsBus bus_;
  std::vector<Joint> joints_;

  std::string port_;
  int baud_rate_ = 1000000;
  int timeout_ms_ = 10;
  bool torque_ = true;
  bool disable_torque_on_deactivate_ = true;
  int acceleration_ = 254;
  double max_velocity_ = 2.0;
  int max_missed_reads_ = 10;
  int return_delay_time_ = -1;
  int p_coefficient_ = -1;
  int d_coefficient_ = -1;
  int i_coefficient_ = -1;
  bool torque_enabled_ = false;
};

}  // namespace so101_scan_hardware
