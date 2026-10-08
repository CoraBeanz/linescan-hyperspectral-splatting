#include "so101_scan_hardware/servo_checks.hpp"

#include <cmath>
#include <cstdio>

#include "so101_scan_hardware/sts_protocol.hpp"

namespace so101_scan_hardware
{
namespace
{

std::string format(const char * fmt, const std::string & name, double a, double b)
{
  char buf[160];
  std::snprintf(buf, sizeof(buf), fmt, name.c_str(), a, b);
  return buf;
}

// Error bits the servo sets for a problem it has detected itself.
constexpr uint8_t kSeriousErrors = sts::error::kOverheat | sts::error::kOverEle | sts::error::kOverload;

std::string error_names(uint8_t bits)
{
  std::string out;
  const std::pair<uint8_t, const char *> names[] = {
    {sts::error::kVoltage, "voltage"}, {sts::error::kAngle, "angle"}, {sts::error::kOverheat, "overheat"},
    {sts::error::kOverEle, "overcurrent"}, {sts::error::kOverload, "overload"}};
  for (const auto & [bit, name] : names) {
    if (bits & bit) {
      out += out.empty() ? name : std::string(", ") + name;
    }
  }
  return out;
}

}  // namespace

void ServoChecks::configure(const ServoLimits & limits, const std::vector<std::string> & names)
{
  limits_ = limits;
  names_ = names;
  timers_.assign(names.size(), Timers{});
}

void ServoChecks::set_supply_voltage(double volts)
{
  supply_voltage_ = volts;
}

double ServoChecks::min_voltage() const
{
  return limits_.min_voltage > 0 ? limits_.min_voltage : 0.8 * supply_voltage_;
}

double ServoChecks::warn_voltage() const
{
  return limits_.warn_voltage > 0 ? limits_.warn_voltage : 0.9 * supply_voltage_;
}

void ServoChecks::clear()
{
  timers_.assign(names_.size(), Timers{});
}

ServoChecks::Result ServoChecks::update(const std::vector<ServoSample> & samples, double dt, bool driving)
{
  Result r;
  const auto & L = limits_;
  const double v_min = min_voltage();
  const double v_warn = warn_voltage();
  // A limit that has held for `hold` seconds: the first one found is the fault.
  auto timed = [&](double & timer, bool over, double hold, const std::string & what) {
      timer = over ? timer + dt : 0.0;
      if (over) {
        r.present.push_back(what);
        if (timer >= hold && r.fault.empty()) {
          r.fault = what;
        }
      }
    };
  for (std::size_t i = 0; i < samples.size() && i < names_.size(); ++i) {
    const auto & s = samples[i];
    auto & t = timers_[i];
    const auto & name = names_[i];
    if (!s.fresh) {
      continue;  // the driver deals with servos that stop answering
    }
    const double load = std::fabs(s.load);
    timed(t.load, load > L.max_load, L.load_time,
      format("%s at %.0f%% of its maximum torque (limit %.0f%%)", name, 100 * load, 100 * L.max_load));
    if (load > L.warn_load && load <= L.max_load) {
      r.warnings.push_back(format("%s at %.0f%% of its maximum torque (warning at %.0f%%)", name, 100 * load,
        100 * L.warn_load));
    }
    timed(t.temperature, s.temperature >= L.max_temperature, L.temperature_time,
      format("%s at %.0f C (limit %.0f C)", name, s.temperature, L.max_temperature));
    if (s.temperature >= L.warn_temperature && s.temperature < L.max_temperature) {
      r.warnings.push_back(format("%s at %.0f C (warning at %.0f C)", name, s.temperature, L.warn_temperature));
    }
    if (v_min > 0) {
      timed(t.voltage, s.voltage < v_min, L.voltage_time,
        format("%s supply at %.1f V (limit %.1f V)", name, s.voltage, v_min));
      if (s.voltage < v_warn && s.voltage >= v_min) {
        r.warnings.push_back(format("%s supply at %.1f V (warning at %.1f V)", name, s.voltage, v_warn));
      }
    }
    // The servo's own alarms: two reads in a row, so one corrupted status byte doesn't stop the arm.
    const uint8_t serious = s.error & (kSeriousErrors | sts::error::kVoltage);
    timed(t.error, serious != 0, 2 * dt, name + " reports " + error_names(serious));
    if (s.error & sts::error::kAngle) {
      r.warnings.push_back(name + " reports its angle limit");
    }
    if (driving) {
      const double error = s.goal - s.position;
      timed(t.stall, std::fabs(error) > L.stall_error && std::fabs(s.velocity) < L.stall_velocity, L.stall_time,
        format("%s stalled %.1f deg short of its goal (limit %.1f deg)", name, std::fabs(error) * 180.0 / M_PI,
        L.stall_error * 180.0 / M_PI));
    } else {
      t.stall = 0.0;
    }
  }
  return r;
}

}  // namespace so101_scan_hardware
