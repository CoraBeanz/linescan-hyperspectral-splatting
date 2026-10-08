#include "so101_scan_hardware/feetech_sts_system.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <limits>
#include <sstream>
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

std::vector<std::string> split(const std::string & s)
{
  std::istringstream in(s);
  std::vector<std::string> out;
  for (std::string w; in >> w; ) {
    out.push_back(w);
  }
  return out;
}

std::string printf_string(const char * fmt, ...) __attribute__((format(printf, 1, 2)));
std::string printf_string(const char * fmt, ...)
{
  char buf[256];
  va_list args;
  va_start(args, fmt);
  std::vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  return buf;
}

}  // namespace

FeetechStsSystem::~FeetechStsSystem()
{
  stop_safety_node();
}

void FeetechStsSystem::stop_safety_node()
{
  // Not under bus_mutex_: a service call waiting for the bus would never let its thread finish.
  safety_node_.reset();
}

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

    // safety: servo limits, see servo_checks.hpp
    auto num = [&](const char * name, double fallback) {
        const auto it = hw.find(name);
        return it == hw.end() ? fallback : std::stod(it->second);
      };
    ServoLimits sl;
    sl.warn_load = num("warn_load", sl.warn_load);
    sl.max_load = num("max_load", sl.max_load);
    sl.load_time = num("load_time", sl.load_time);
    sl.warn_temperature = num("warn_temperature", sl.warn_temperature);
    sl.max_temperature = num("max_temperature", sl.max_temperature);
    sl.temperature_time = num("temperature_time", sl.temperature_time);
    sl.min_voltage = num("min_voltage", sl.min_voltage);
    sl.warn_voltage = num("warn_voltage", sl.warn_voltage);
    sl.voltage_time = num("voltage_time", sl.voltage_time);
    sl.stall_error = num("stall_error", sl.stall_error);
    sl.stall_velocity = num("stall_velocity", sl.stall_velocity);
    sl.stall_time = num("stall_time", sl.stall_time);
    std::vector<std::string> names;
    for (const auto & ji : info_.joints) {
      names.push_back(ji.name);
    }
    checks_.configure(sl, names);
    torque_off_after_ = num("torque_off_after", torque_off_after_);
    resume_tolerance_ = num("resume_tolerance", resume_tolerance_);
    safety_node_enabled_ = to_bool(param(hw, "safety_node", "true"));

    // soft limits, see arm_model.hpp
    soft_limits_ = to_bool(param(hw, "soft_limits", "true"));
    workspace_.table_z = num("table_z", workspace_.table_z);
    workspace_.table_clearance = num("table_clearance", workspace_.table_clearance);
    workspace_.base_keepout_radius = num("base_keepout_radius", workspace_.base_keepout_radius);
    workspace_.base_keepout_top = num("base_keepout_top", workspace_.base_keepout_top);
    stall_torque_ = num("stall_torque", stall_torque_);
    warn_gravity_load_ = num("warn_gravity_load", warn_gravity_load_);
    max_gravity_load_ = num("max_gravity_load", max_gravity_load_);
    if (soft_limits_) {
      // fails closed: without the arm's model there are no soft limits to keep to
      model_.load(info_.original_xml, names,
        split(param(hw, "workspace_links", "upper_arm_link lower_arm_link wrist_link wrist_roll_link")),
        param(hw, "workspace_box_link", "scan_head_link"));
    }
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
  std::vector<double> volts;
  for (const auto & j : joints_) {
    RCLCPP_INFO(
      logger_, "%-14s id %d: %7.2f deg, %.1f V, %.0f C", j.name.c_str(), j.id, j.position * 180.0 / M_PI,
      j.voltage, j.temperature);
    volts.push_back(j.voltage);
  }
  // The supply at rest, which the automatic voltage limits are a fraction of.
  std::nth_element(volts.begin(), volts.begin() + volts.size() / 2, volts.end());
  checks_.set_supply_voltage(volts[volts.size() / 2]);
  const auto & sl = checks_.limits();
  RCLCPP_INFO(
    logger_, "Safety limits: load %.0f%% for %.1f s, %.0f C, %.1f V (supply %.1f V), stall %.1f deg for %.1f s; "
    "motors off after %.1f s; soft limits %s", 100 * sl.max_load, sl.load_time, sl.max_temperature,
    checks_.min_voltage(), checks_.supply_voltage(), sl.stall_error * 180 / M_PI, sl.stall_time, torque_off_after_,
    soft_limits_ ? "on" : "off");
  if (soft_limits_) {
    RCLCPP_INFO(
      logger_, "Soft limits: %.0f mm above the table (z = %.4f m), out of a %.0f mm cylinder around the base up to "
      "z = %.3f m, gravity at most %.0f%% of %.2f N m", workspace_.table_clearance * 1e3, workspace_.table_z,
      workspace_.base_keepout_radius * 1e3, workspace_.base_keepout_top, 100 * max_gravity_load_, stall_torque_);
  }
  if (safety_node_enabled_ && !safety_node_ && rclcpp::ok()) {
    ArmSafetyNode::Callbacks cb;
    cb.stop = [this](bool torque_off, const std::string & reason) {return stop(torque_off, reason);};
    cb.reset = [this]() {return reset();};
    cb.status = [this]() {return safety_status();};
    safety_node_ = std::make_unique<ArmSafetyNode>(cb);
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
    checks_.clear();
    reason_.clear();
    soft_limit_.clear();
    fault_time_ = 0.0;
    if (!torque_) {
      // The servos may still be holding from the last run, which leaves them on.
      set_torque(false);
      state_ = SafetyState::kReadOnly;
      RCLCPP_WARN(logger_, "Torque off (torque:=false): the arm is limp and can be moved by hand; positions are only read");
      return CallbackReturn::SUCCESS;
    }
    // A joint well outside its calibrated range means the calibration doesn't fit the arm, and
    // driving it back into range could swing the arm hard: leave the motors as they are.
    const std::string outside = outside_range();
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
    state_ = SafetyState::kOk;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "%s", e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(logger_, "Torque on, holding the current pose");
  if (model_.loaded() && soft_limits_) {
    const auto e = model_.evaluate(sent(), workspace_);
    if (e.workspace_margin < 0) {
      RCLCPP_WARN(
        logger_, "The arm starts outside its workspace (%s %.0f mm past the %s limit): it can only move back in",
        e.workspace_part.c_str(), -e.workspace_margin * 1e3, e.workspace_limit.c_str());
    }
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn FeetechStsSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(bus_mutex_);
  state_ = SafetyState::kInactive;
  update_status();
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
  {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    state_ = SafetyState::kInactive;
    bus_.close();
  }
  stop_safety_node();
  return CallbackReturn::SUCCESS;
}

CallbackReturn FeetechStsSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
  {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    state_ = SafetyState::kInactive;
    try {
      if (bus_.is_open() && torque_enabled_ && disable_torque_on_deactivate_) {
        set_torque(false);
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "%s", e.what());
    }
    bus_.close();
  }
  stop_safety_node();
  return CallbackReturn::SUCCESS;
}

CallbackReturn FeetechStsSystem::on_error(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(bus_mutex_);
  // The servos keep their last goal and torque: dropping the arm is worse than
  // leaving it where it is.
  RCLCPP_ERROR(logger_, "Stopped after an error; the servos keep their last goal until their power is cut");
  state_ = SafetyState::kInactive;
  reason_ = "the driver stopped after an error";
  update_status();
  bus_.close();
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type FeetechStsSystem::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  std::lock_guard<std::mutex> lock(bus_mutex_);
  try {
    read_all(false);
    if (state_ != SafetyState::kInactive) {
      check_servos(std::clamp(period.seconds(), 0.001, 0.1));
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "%s", e.what());
    return hardware_interface::return_type::ERROR;
  }
  update_status();
  for (const auto & j : joints_) {
    if (j.missed > max_missed_reads_) {
      RCLCPP_ERROR(logger_, "%s (id %d) stopped answering", j.name.c_str(), j.id);
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

void FeetechStsSystem::check_servos(double dt)
{
  std::vector<ServoSample> samples(joints_.size());
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const auto & j = joints_[i];
    auto & s = samples[i];
    s.fresh = j.missed == 0;
    s.position = j.position;
    s.velocity = j.velocity;
    s.load = j.load;
    s.voltage = j.voltage;
    s.temperature = j.temperature;
    s.goal = j.sent;
    s.error = bus_.last_error(j.id);
  }
  const bool driving = torque_enabled_ && state_ != SafetyState::kReadOnly && state_ != SafetyState::kTorqueOff;
  auto r = checks_.update(samples, dt, driving);
  warnings_ = r.warnings;
  present_ = r.present;
  switch (state_) {
    case SafetyState::kOk:
    case SafetyState::kResuming:
      if (!r.fault.empty()) {
        stop_locked(false, r.fault);
      }
      break;
    case SafetyState::kHolding:
      // Holding stops a stall at once, and takes the strain off; a cause that is still there
      // after torque_off_after seconds of it is one holding doesn't fix.
      fault_time_ = r.present.empty() ? 0.0 : fault_time_ + dt;
      if (torque_off_after_ > 0 && fault_time_ >= torque_off_after_) {
        stop_locked(true, printf_string("still %s after %.1f s holding", r.present.front().c_str(), fault_time_));
      }
      break;
    case SafetyState::kReadOnly:
      // nothing to stop: say so instead
      for (const auto & p : r.present) {
        warnings_.push_back("over the limit: " + p);
      }
      break;
    default:
      break;
  }
}

hardware_interface::return_type FeetechStsSystem::write(const rclcpp::Time &, const rclcpp::Duration & period)
{
  std::lock_guard<std::mutex> lock(bus_mutex_);
  if (!torque_enabled_) {
    return hardware_interface::return_type::OK;
  }
  if (state_ == SafetyState::kResuming) {
    // After a reset the commands count again only once they ask for where the arm is: a
    // controller still holding an old goal would otherwise move it there.
    bool caught_up = true;
    for (const auto & j : joints_) {
      const auto [lo, hi] = limits(j);
      caught_up = caught_up && std::isfinite(j.command) &&
        std::fabs(std::clamp(j.command, lo, hi) - j.sent) <= resume_tolerance_;
    }
    if (caught_up) {
      state_ = SafetyState::kOk;
      RCLCPP_INFO(logger_, "Following the controller again");
    }
  }
  if (state_ == SafetyState::kOk) {
    // A controller can ask for a jump, or for more than the calibrated range: the arm only
    // gets max_velocity of it per cycle, and only within the range.
    const double dt = std::clamp(period.seconds(), 0.001, 0.1);
    const double max_step = max_velocity_ * dt;
    std::vector<double> next = sent();
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      const auto & j = joints_[i];
      if (std::isfinite(j.command)) {
        const auto [lo, hi] = limits(j);
        next[i] += std::clamp(std::clamp(j.command, lo, hi) - j.sent, -max_step, max_step);
      }
    }
    // ... and only as far as the soft limits allow: a step out of the workspace or past the
    // gravity budget isn't taken (one back in always is).
    std::string why;
    if (model_.loaded() && soft_limits_) {
      why = soft_limit(model_.evaluate(next, workspace_), model_.evaluate(sent(), workspace_));
    }
    if (why.empty()) {
      for (std::size_t i = 0; i < joints_.size(); ++i) {
        joints_[i].sent = next[i];
      }
    } else if (why != soft_limit_) {
      RCLCPP_WARN(logger_, "Holding back: %s", why.c_str());
    }
    soft_limit_ = why;
  }
  std::vector<std::pair<uint8_t, std::vector<uint8_t>>> goal;
  for (const auto & j : joints_) {
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

std::vector<double> FeetechStsSystem::sent() const
{
  std::vector<double> out;
  for (const auto & j : joints_) {
    out.push_back(j.sent);
  }
  return out;
}

std::string FeetechStsSystem::soft_limit(const ArmModel::Evaluation & to, const ArmModel::Evaluation & from) const
{
  if (to.workspace_margin < 0 && to.workspace_margin < from.workspace_margin) {
    if (to.workspace_limit == "table") {
      const double gap = workspace_.table_clearance + to.workspace_margin;
      if (gap < 0) {
        return printf_string("the %s would go %.1f mm below the table top", to.workspace_part.c_str(), -gap * 1e3);
      }
      return printf_string("the %s would come within %.1f mm of the table (keeps %.0f mm)", to.workspace_part.c_str(),
               gap * 1e3, workspace_.table_clearance * 1e3);
    }
    return printf_string("the %s would go %.0f mm into the space kept clear around the base", to.workspace_part.c_str(),
             -to.workspace_margin * 1e3);
  }
  for (std::size_t k = 0; k < to.torque.size(); ++k) {
    const double load = std::fabs(to.torque[k]) / stall_torque_;
    if (load > max_gravity_load_ && load > std::fabs(from.torque[k]) / stall_torque_) {
      return printf_string("%s would hold %.1f%% of its stall torque against gravity (limit %.0f%%)",
               joints_[k].name.c_str(), 100 * load, 100 * max_gravity_load_);
    }
  }
  return "";
}

std::string FeetechStsSystem::outside_range() const
{
  std::string outside;
  for (const auto & j : joints_) {
    if (j.ticks < j.min_ticks - kRangeSlackTicks || j.ticks > j.max_ticks + kRangeSlackTicks) {
      outside += " " + j.name + " at " + std::to_string(j.ticks) + " ticks (range " +
        std::to_string(j.min_ticks) + ".." + std::to_string(j.max_ticks) + ")";
    }
  }
  return outside;
}

std::pair<bool, std::string> FeetechStsSystem::stop(bool torque_off, const std::string & reason)
{
  std::lock_guard<std::mutex> lock(bus_mutex_);
  auto out = stop_locked(torque_off, reason);
  update_status();
  return out;
}

std::pair<bool, std::string> FeetechStsSystem::stop_locked(bool torque_off, const std::string & reason)
{
  if (state_ == SafetyState::kInactive) {
    return {false, "the arm driver isn't active"};
  }
  if (state_ == SafetyState::kReadOnly) {
    return {true, "the motors are off already (torque:=false)"};
  }
  const bool was_stopped = state_ == SafetyState::kHolding || state_ == SafetyState::kTorqueOff;
  if (state_ == SafetyState::kTorqueOff || (was_stopped && !torque_off)) {
    return {true, "already stopped: " + reason_};
  }
  try {
    if (torque_off) {
      set_torque(false);
      state_ = SafetyState::kTorqueOff;
    } else {
      // Hold where the arm is now, not where it was told to go: a joint pressing on something
      // stops pressing.
      std::vector<std::pair<uint8_t, std::vector<uint8_t>>> goal;
      for (auto & j : joints_) {
        j.sent = j.position;
        goal.push_back({j.id, sts::to_le16(static_cast<uint16_t>(std::clamp(j.ticks, 0, sts::kTicksPerTurn - 1)))});
      }
      bus_.sync_write(sts::reg::kGoalPosition, 2, goal);
      state_ = SafetyState::kHolding;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Stopping the arm: %s", e.what());
    return {false, std::string("couldn't reach the servos: ") + e.what()};
  }
  reason_ = was_stopped ? reason_ + "; then " + reason : reason;
  fault_time_ = 0.0;
  const char * how = torque_off ? "Motors off, the arm is limp" : "Holding the arm where it is";
  RCLCPP_ERROR(logger_, "Stopped: %s. %s; /arm_safety/reset lets it move again.", reason.c_str(), how);
  return {true, std::string(how) + ": " + reason};
}

std::pair<bool, std::string> FeetechStsSystem::reset()
{
  std::lock_guard<std::mutex> lock(bus_mutex_);
  switch (state_) {
    case SafetyState::kOk:
      return {true, "nothing to reset: the arm isn't stopped"};
    case SafetyState::kResuming:
      return {true, "reset already; holding until a controller asks for the pose the arm is in"};
    case SafetyState::kReadOnly:
      return {false, "the motors are off (torque:=false); relaunch with the motors on"};
    case SafetyState::kInactive:
      return {false, "the arm driver isn't active"};
    default:
      break;
  }
  if (!present_.empty()) {
    return {false, "not reset, still: " + present_.front()};
  }
  try {
    if (state_ == SafetyState::kTorqueOff) {
      // Like activating: hold where the arm is now, unless it is far outside its range.
      const std::string outside = outside_range();
      if (!outside.empty()) {
        return {false, "not reset, outside the calibrated range:" + outside +
                ". Move it back by hand, or relaunch with torque:=false"};
      }
      std::vector<std::pair<uint8_t, std::vector<uint8_t>>> goal;
      for (const auto & j : joints_) {
        goal.push_back({j.id, sts::to_le16(static_cast<uint16_t>(j.ticks))});
      }
      bus_.sync_write(sts::reg::kGoalPosition, 2, goal);
      set_torque(true);
    }
  } catch (const std::exception & e) {
    return {false, std::string("couldn't reach the servos: ") + e.what()};
  }
  for (auto & j : joints_) {
    j.sent = j.position;
    // a controller that starts now (arm_controller restarted) starts from here
    j.command = j.position;
  }
  checks_.clear();
  fault_time_ = 0.0;
  soft_limit_.clear();
  RCLCPP_INFO(logger_, "Reset after: %s. Holding until a controller asks for this pose", reason_.c_str());
  reason_.clear();
  state_ = SafetyState::kResuming;
  update_status();
  return {true, "reset; the arm holds where it is until a controller asks for that pose (restarting "
          "arm_controller does)"};
}

void FeetechStsSystem::update_status()
{
  SafetyStatus s;
  s.state = state_;
  s.reason = reason_;
  s.warnings = warnings_;
  if (!soft_limit_.empty()) {
    s.warnings.push_back("soft limit: " + soft_limit_);
  }
  std::vector<double> torque;
  if (model_.loaded()) {
    const auto e = model_.evaluate(sent(), workspace_);
    s.workspace_margin = e.workspace_margin;
    torque = e.torque;
  }
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const auto & j = joints_[i];
    s.joint_names.push_back(j.name);
    s.load.push_back(j.load);
    s.temperature.push_back(j.temperature);
    s.voltage.push_back(j.voltage);
    s.tracking_error.push_back(j.sent - j.position);
    const double g = i < torque.size() ? std::fabs(torque[i]) / stall_torque_ : 0.0;
    s.gravity_load.push_back(g);
    if (g > warn_gravity_load_ && soft_limit_.empty()) {
      s.warnings.push_back(printf_string("%s holds %.0f%% of its stall torque against gravity (warning at %.0f%%)",
        j.name.c_str(), 100 * g, 100 * warn_gravity_load_));
    }
  }
  std::lock_guard<std::mutex> lock(status_mutex_);
  status_ = std::move(s);
}

SafetyStatus FeetechStsSystem::safety_status()
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  return status_;
}

}  // namespace so101_scan_hardware

PLUGINLIB_EXPORT_CLASS(so101_scan_hardware::FeetechStsSystem, hardware_interface::SystemInterface)
