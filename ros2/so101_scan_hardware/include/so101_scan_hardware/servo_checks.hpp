// Thresholds on what the servos report, checked by the driver every control cycle.
//
// Each check has a warning level, which only gets reported, and a limit, which stops the arm
// once it has held for a while (so a load spike while accelerating doesn't). Stall detection
// looks for a joint that is far from the goal it was sent and not moving towards it: the head
// pressing on the table, a cable caught, or a joint that can't hold the weight.
//
// The servos protect themselves too (an STS3215 cuts its torque at 70 C, or when overloaded
// for long enough), but one joint at a time and without warning; these limits come first.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace so101_scan_hardware
{

struct ServoLimits
{
  double warn_load = 0.6;          // fraction of the servo's maximum torque
  double max_load = 0.9;
  double load_time = 1.0;          // s above max_load before stopping
  double warn_temperature = 55.0;  // deg C
  double max_temperature = 65.0;
  double temperature_time = 1.0;   // s
  double min_voltage = 0.0;        // V; 0: 80% of what the supply gave when the driver started
  double warn_voltage = 0.0;       // V; 0: 90% of it
  double voltage_time = 0.5;       // s
  double stall_error = 0.2;        // rad between the goal sent and where the joint is
  double stall_velocity = 0.1;     // rad/s: slower than this counts as not moving
  double stall_time = 0.5;         // s
};

// What one servo reported in the last cycle, and the goal it was sent.
struct ServoSample
{
  bool fresh = false;   // it answered this cycle
  double position = 0.0;
  double velocity = 0.0;
  double load = 0.0;
  double voltage = 0.0;
  double temperature = 0.0;
  double goal = 0.0;    // rad, as last sent
  uint8_t error = 0;    // the error bits of its status packet
};

class ServoChecks
{
public:
  void configure(const ServoLimits & limits, const std::vector<std::string> & names);
  const ServoLimits & limits() const {return limits_;}
  // Fills in the automatic voltage thresholds from what the servos report with the arm at rest.
  void set_supply_voltage(double volts);
  double supply_voltage() const {return supply_voltage_;}
  double min_voltage() const;
  double warn_voltage() const;

  struct Result
  {
    std::string fault;                  // a limit that has held long enough: stop the arm
    std::vector<std::string> present;   // limits passed right now, however briefly
    std::vector<std::string> warnings;  // warning levels passed
  };
  // driving: the motors are on and following goals, so a stall means something.
  Result update(const std::vector<ServoSample> & samples, double dt, bool driving);
  // Starts every timer again, e.g. after a reset.
  void clear();

private:
  struct Timers
  {
    double load = 0, temperature = 0, voltage = 0, stall = 0, error = 0;
  };
  ServoLimits limits_;
  std::vector<std::string> names_;
  std::vector<Timers> timers_;
  double supply_voltage_ = 0.0;
};

}  // namespace so101_scan_hardware
