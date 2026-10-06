#include "so101_scan_hardware/feetech_sts_system.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace so101_scan_hardware
{
namespace
{

using hardware_interface::CallbackReturn;

constexpr double kRadPerTick = 2.0 * M_PI / sts::kTicksPerTurn;
// One sync read covers Present_Position .. Present_Temperature.
constexpr uint8_t kStateAddress = sts::reg::kPresentPosition;
constexpr uint8_t kStateLength = 8;

std::string param(
  const std::unordered_map<std::string, std::string> & params, const std::string & name,
  const std::string & fallback)
{
  auto it = params.find(name);
  return it == params.end() ? fallback : it->second;
}

bool to_bool(const std::string & s)
{
  std::string v = s;
  std::transform(v.begin(), v.end(), v.begin(), ::tolower);
  return v == "true" || v == "1" || v == "yes";
}

}  // namespace

double FeetechStsSystem::ticks_to_rad(const Joint & j, double ticks)
{
  return j.sign * (ticks - j.zero_ticks) * kRadPerTick;
}

int FeetechStsSystem::rad_to_ticks(const Joint & j, double rad)
{
  const double ticks = j.zero_ticks + j.sign * rad / kRadPerTick;
  return static_cast<int>(std::lround(std::clamp(ticks, 0.0, sts::kTicksPerTurn - 1.0)));
}

std::pair<double, double> FeetechStsSystem::limits(const Joint & j)
{
  const double a = ticks_to_rad(j, j.min_ticks);
  const double b = ticks_to_rad(j, j.max_ticks);
  return {std::min(a, b), std::max(a, b)};
}

CallbackReturn FeetechStsSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  const auto & hw = info_.hardware_parameters;
  try {
    port_ = param(hw, "port", "");
    baud_rate_ = std::stoi(param(hw, "baud_rate", "1000000"));
    timeout_ms_ = std::stoi(param(hw, "timeout_ms", "10"));
    torque_ = to_bool(param(hw, "torque", "true"));
    disable_torque_on_deactivate_ = to_bool(param(hw, "disable_torque_on_deactivate", "false"));
    acceleration_ = std::stoi(param(hw, "acceleration", "254"));
    max_velocity_ = std::stod(param(hw, "max_velocity", "2.0"));
    max_missed_reads_ = std::stoi(param(hw, "max_missed_reads", "10"));
    return_delay_time_ = std::stoi(param(hw, "return_delay_time", "-1"));
    p_coefficient_ = std::stoi(param(hw, "p_coefficient", "-1"));
    d_coefficient_ = std::stoi(param(hw, "d_coefficient", "-1"));
    i_coefficient_ = std::stoi(param(hw, "i_coefficient", "-1"));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Bad hardware parameter: %s", e.what());
    return CallbackReturn::ERROR;
  }
  if (port_.empty()) {
    RCLCPP_ERROR(logger_, "The 'port' hardware parameter is empty: set it to the servo bus adapter, e.g. /dev/so101");
    return CallbackReturn::ERROR;
  }

  joints_.clear();
  for (const auto & ji : info_.joints) {
    Joint j;
    j.name = ji.name;
    try {
      j.id = static_cast<uint8_t>(std::stoi(param(ji.parameters, "id", "")));
      j.zero_ticks = std::stod(param(ji.parameters, "zero_ticks", ""));
      j.sign = std::stoi(param(ji.parameters, "sign", "1")) < 0 ? -1 : 1;
      j.min_ticks = std::stoi(param(ji.parameters, "min_ticks", "0"));
      j.max_ticks = std::stoi(param(ji.parameters, "max_ticks", "4095"));
    } catch (const std::exception &) {
      RCLCPP_ERROR(
        logger_, "Joint '%s' needs 'id' and 'zero_ticks' parameters (from the calibration file)", j.name.c_str());
      return CallbackReturn::ERROR;
    }
    if (j.min_ticks >= j.max_ticks) {
      RCLCPP_ERROR(logger_, "Joint '%s': min_ticks must be below max_ticks", j.name.c_str());
      return CallbackReturn::ERROR;
    }
    if (ji.command_interfaces.size() != 1 ||
      ji.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(logger_, "Joint '%s' must have exactly one command interface, 'position'", j.name.c_str());
      return CallbackReturn::ERROR;
    }
    for (const auto & si : ji.state_interfaces) {
      if (si.name != hardware_interface::HW_IF_POSITION && si.name != hardware_interface::HW_IF_VELOCITY &&
        si.name != "load" && si.name != "voltage" && si.name != "temperature")
      {
        RCLCPP_ERROR(
          logger_, "Joint '%s': unknown state interface '%s' (have position, velocity, load, voltage, temperature)",
          j.name.c_str(), si.name.c_str());
        return CallbackReturn::ERROR;
      }
    }
    j.command = std::numeric_limits<double>::quiet_NaN();
    joints_.push_back(j);
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FeetechStsSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    auto & j = joints_[i];
    for (const auto & si : info_.joints[i].state_interfaces) {
      double * value = nullptr;
      if (si.name == hardware_interface::HW_IF_POSITION) {
        value = &j.position;
      } else if (si.name == hardware_interface::HW_IF_VELOCITY) {
        value = &j.velocity;
      } else if (si.name == "load") {
        value = &j.load;
      } else if (si.name == "voltage") {
        value = &j.voltage;
      } else if (si.name == "temperature") {
        value = &j.temperature;
      }
      out.emplace_back(j.name, si.name, value);
    }
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> FeetechStsSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  for (auto & j : joints_) {
    out.emplace_back(j.name, hardware_interface::HW_IF_POSITION, &j.command);
  }
  return out;
}

std::vector<uint8_t> FeetechStsSystem::ids() const
{
  std::vector<uint8_t> out;
  for (const auto & j : joints_) {
    out.push_back(j.id);
  }
  return out;
}

bool FeetechStsSystem::configure_servo(const Joint & j)
{
  bool ok = true;
  auto model = bus_.read_u16(j.id, sts::reg::kModelNumber);
  if (model && *model != sts::kStsModelNumber) {
    RCLCPP_WARN(logger_, "Servo %d (%s) reports model %d, not an STS3215 (777)", j.id, j.name.c_str(), *model);
  }
  auto mode = bus_.read_u8(j.id, sts::reg::kOperatingMode);
  if (mode && *mode != 0) {
    RCLCPP_ERROR(
      logger_, "Servo %d (%s) is in operating mode %d, not position mode (0). Run the calibration tool, which sets it.",
      j.id, j.name.c_str(), *mode);
    return false;
  }
  // EEPROM settings: only written when they differ, so the EEPROM isn't worn
  // out by rewriting the same values on every start.
  struct Setting {uint8_t address; int value; const char * name;};
  const Setting settings[] = {
    {sts::reg::kReturnDelayTime, return_delay_time_, "return delay time"},
    {sts::reg::kPCoefficient, p_coefficient_, "P coefficient"},
    {sts::reg::kDCoefficient, d_coefficient_, "D coefficient"},
    {sts::reg::kICoefficient, i_coefficient_, "I coefficient"},
  };
  for (const auto & s : settings) {
    if (s.value < 0) {
      continue;
    }
    auto now = bus_.read_u8(j.id, s.address);
    if (!now) {
      RCLCPP_ERROR(logger_, "Servo %d (%s): can't read its %s", j.id, j.name.c_str(), s.name);
      ok = false;
      continue;
    }
    if (*now == s.value) {
      continue;
    }
    RCLCPP_INFO(logger_, "Servo %d (%s): setting %s %d -> %d", j.id, j.name.c_str(), s.name, *now, s.value);
    ok = bus_.write_u8(j.id, sts::reg::kLock, 0) && ok;
    ok = bus_.write_u8(j.id, s.address, static_cast<uint8_t>(s.value)) && ok;
    ok = bus_.write_u8(j.id, sts::reg::kLock, 1) && ok;
  }
  return ok;
}

CallbackReturn FeetechStsSystem::on_configure(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(bus_mutex_);
  try {
    bus_.open(port_, baud_rate_, timeout_ms_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "%s", e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(logger_, "Opened %s at %d baud", port_.c_str(), baud_rate_);

  std::string missing;
  for (const auto & j : joints_) {
    bool found = false;
    for (int attempt = 0; attempt < 3 && !found; ++attempt) {
      found = bus_.ping(j.id);
    }
    if (!found) {
      missing += " " + j.name + " (id " + std::to_string(j.id) + ")";
    }
  }
  if (!missing.empty()) {
    RCLCPP_ERROR(
      logger_, "No answer from:%s. Check the servo power supply and the cable, and that the IDs match.",
      missing.c_str());
    bus_.close();
    return CallbackReturn::ERROR;
  }
  for (const auto & j : joints_) {
    if (!configure_servo(j)) {
      bus_.close();
      return CallbackReturn::ERROR;
    }
  }
  if (!read_all(true)) {
    RCLCPP_ERROR(logger_, "The first sync read failed");
    bus_.close();
    return CallbackReturn::ERROR;
  }
  for (const auto & j : joints_) {
    RCLCPP_INFO(
      logger_, "%-14s id %d: %7.2f deg, %.1f V, %.0f C", j.name.c_str(), j.id, j.position * 180.0 / M_PI,
      j.voltage, j.temperature);
  }
  return CallbackReturn::SUCCESS;
}

bool FeetechStsSystem::read_all(bool update_commands)
{
  const auto data = bus_.sync_read(kStateAddress, kStateLength, ids());
  bool all = true;
  for (auto & j : joints_) {
    auto it = data.find(j.id);
    if (it == data.end()) {
      ++j.missed;
      all = false;
      continue;
    }
    const uint8_t * d = it->second.data();
    j.missed = 0;
    j.ticks = sts::le16(d);
    j.position = ticks_to_rad(j, j.ticks);
    j.velocity = j.sign * sts::decode_signed(sts::le16(d + 2), 15) * kRadPerTick;
    j.load = j.sign * sts::decode_signed(sts::le16(d + 4), 10) / 1000.0;
    j.voltage = d[6] / 10.0;
    j.temperature = d[7];
    if (update_commands) {
      j.sent = j.position;
    }
  }
  return all;
}

void FeetechStsSystem::set_torque(bool on)
{
  std::vector<std::pair<uint8_t, std::vector<uint8_t>>> data;
  for (const auto & j : joints_) {
    data.push_back({j.id, {static_cast<uint8_t>(on ? 1 : 0)}});
  }
  bus_.sync_write(sts::reg::kTorqueEnable, 1, data);
  torque_enabled_ = on;
}

CallbackReturn FeetechStsSystem::on_activate(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(bus_mutex_);
  try {
    if (!read_all(true)) {
      RCLCPP_ERROR(logger_, "Can't read every servo, not activating");
      return CallbackReturn::ERROR;
    }
    for (auto & j : joints_) {
      j.command = j.position;
    }
    if (!torque_) {
      // The servos may still be holding from the last run, which leaves them on.
      set_torque(false);
      RCLCPP_WARN(logger_, "Torque off (torque:=false): the arm is limp and can be moved by hand; positions are only read");
      return CallbackReturn::SUCCESS;
    }
    // A joint well outside its calibrated range means the calibration doesn't fit the arm, and
    // driving it back into range could swing the arm hard: leave the motors as they are.
    std::string outside;
    for (const auto & j : joints_) {
      if (j.ticks < j.min_ticks - kRangeSlackTicks || j.ticks > j.max_ticks + kRangeSlackTicks) {
        outside += " " + j.name + " at " + std::to_string(j.ticks) + " ticks (range " +
          std::to_string(j.min_ticks) + ".." + std::to_string(j.max_ticks) + ")";
      }
    }
    if (!outside.empty()) {
      RCLCPP_ERROR(
        logger_, "Not switching the motors on, outside the calibrated range:%s. If the calibration is old, run "
        "sts_calibrate again; otherwise launch with torque:=false and move the joint into its range by hand.",
        outside.c_str());
      return CallbackReturn::FAILURE;
    }
    // Hold exactly where the arm is now, then switch the motors on. A joint a little past its
    // range is brought back into it by write(), at max_velocity.
    std::vector<std::pair<uint8_t, std::vector<uint8_t>>> accel, goal;
    for (const auto & j : joints_) {
      accel.push_back({j.id, {static_cast<uint8_t>(std::clamp(acceleration_, 0, 254))}});
      goal.push_back({j.id, sts::to_le16(static_cast<uint16_t>(j.ticks))});
    }
    bus_.sync_write(sts::reg::kAcceleration, 1, accel);
    bus_.sync_write(sts::reg::kGoalPosition, 2, goal);
    set_torque(true);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "%s", e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(logger_, "Torque on, holding the current pose");
  return CallbackReturn::SUCCESS;
}

CallbackReturn FeetechStsSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(bus_mutex_);
  try {
    if (torque_enabled_ && disable_torque_on_deactivate_) {
      set_torque(false);
      RCLCPP_INFO(logger_, "Torque off");
    } else if (torque_enabled_) {
      RCLCPP_INFO(logger_, "The servos keep holding the arm; torque:=false or cutting their power lets it go");
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "%s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn FeetechStsSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(bus_mutex_);
  bus_.close();
  return CallbackReturn::SUCCESS;
}

CallbackReturn FeetechStsSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(bus_mutex_);
  try {
    if (bus_.is_open() && torque_enabled_ && disable_torque_on_deactivate_) {
      set_torque(false);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "%s", e.what());
  }
  bus_.close();
  return CallbackReturn::SUCCESS;
}

CallbackReturn FeetechStsSystem::on_error(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(bus_mutex_);
  // The servos keep their last goal and torque: dropping the arm is worse than
  // leaving it where it is.
  RCLCPP_ERROR(logger_, "Stopped after an error; the servos keep their last goal until their power is cut");
  bus_.close();
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type FeetechStsSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::lock_guard<std::mutex> lock(bus_mutex_);
  try {
    read_all(false);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "%s", e.what());
    return hardware_interface::return_type::ERROR;
  }
  for (const auto & j : joints_) {
    if (j.missed > max_missed_reads_) {
      RCLCPP_ERROR(logger_, "%s (id %d) stopped answering", j.name.c_str(), j.id);
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FeetechStsSystem::write(const rclcpp::Time &, const rclcpp::Duration & period)
{
  std::lock_guard<std::mutex> lock(bus_mutex_);
  if (!torque_enabled_) {
    return hardware_interface::return_type::OK;
  }
  // A controller can ask for a jump, or for more than the calibrated range: the arm only
  // gets max_velocity of it per cycle, and only within the range.
  const double dt = std::clamp(period.seconds(), 0.001, 0.1);
  const double max_step = max_velocity_ * dt;
  std::vector<std::pair<uint8_t, std::vector<uint8_t>>> goal;
  for (auto & j : joints_) {
    if (std::isfinite(j.command)) {
      const auto [lo, hi] = limits(j);
      j.sent += std::clamp(std::clamp(j.command, lo, hi) - j.sent, -max_step, max_step);
    }
    goal.push_back({j.id, sts::to_le16(static_cast<uint16_t>(rad_to_ticks(j, j.sent)))});
  }
  try {
    bus_.sync_write(sts::reg::kGoalPosition, 2, goal);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "%s", e.what());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace so101_scan_hardware

PLUGINLIB_EXPORT_CLASS(so101_scan_hardware::FeetechStsSystem, hardware_interface::SystemInterface)
