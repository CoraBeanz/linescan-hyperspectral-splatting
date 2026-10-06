// The real driver, end to end, against the Python fake bus on a pseudo-terminal.
#include <gtest/gtest.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "so101_scan_hardware/feetech_sts_system.hpp"
#include "so101_scan_hardware/sts_bus.hpp"

#ifndef PYTHON_EXECUTABLE
#define PYTHON_EXECUTABLE "python3"
#endif

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using so101_scan_hardware::FeetechStsSystem;
using so101_scan_hardware::StsBus;
namespace reg = so101_scan_hardware::sts::reg;
using namespace std::chrono_literals;

namespace
{

const rclcpp_lifecycle::State kUnconfigured(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
const rclcpp_lifecycle::State kInactive(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
const rclcpp_lifecycle::State kActive(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");

std::string urdf(const std::string & port, const std::string & extra_joint = "", const std::string & torque = "true")
{
  std::string joints;
  const std::vector<std::pair<std::string, int>> arm = {
    {"shoulder_pan", 1}, {"shoulder_lift", 2}, {"elbow_flex", 3}, {"wrist_flex", 4}, {"wrist_roll", 5}};
  auto joint = [](const std::string & name, int id) {
      return "<joint name='" + name + "'><param name='id'>" + std::to_string(id) +
             "</param><param name='zero_ticks'>2048</param><param name='sign'>1</param>"
             "<param name='min_ticks'>1000</param><param name='max_ticks'>3000</param>"
             "<command_interface name='position'/><state_interface name='position'/>"
             "<state_interface name='velocity'/><state_interface name='load'/>"
             "<state_interface name='voltage'/><state_interface name='temperature'/></joint>";
    };
  for (const auto & [name, id] : arm) {
    joints += joint(name, id);
  }
  if (!extra_joint.empty()) {
    joints += joint(extra_joint, 9);
  }
  return "<?xml version='1.0'?><robot name='t'><ros2_control name='arm' type='system'><hardware>"
         "<plugin>so101_scan_hardware/FeetechStsSystem</plugin>"
         "<param name='port'>" + port + "</param><param name='torque'>" + torque + "</param>"
         "<param name='max_velocity'>3.0</param><param name='p_coefficient'>16</param>"
         "<param name='return_delay_time'>0</param></hardware>" + joints + "</ros2_control></robot>";
}

constexpr double kRadPerTick = 2 * M_PI / 4096;

double state(const std::vector<hardware_interface::StateInterface> & ifs, const std::string & name)
{
  for (const auto & s : ifs) {
    if (s.get_name() == name) {
      return s.get_value();
    }
  }
  ADD_FAILURE() << "no state interface " << name;
  return NAN;
}

class FakeBusTest : public ::testing::Test
{
protected:
  // Starts the fake bus with five servos; ticks are start positions from id 1 on (default 2048).
  void start_bus(const std::vector<int> & ticks = {})
  {
    link_ = "/tmp/so101_test_bus_" + std::to_string(getpid());
    unlink(link_.c_str());  // a stale link would look like the new bus has started
    std::vector<std::string> args = {PYTHON_EXECUTABLE, "-m", "so101_scan_hardware.fake_bus", "--link", link_};
    if (!ticks.empty()) {
      args.push_back("--ticks");
      for (int t : ticks) {
        args.push_back(std::to_string(t));
      }
    }
    std::vector<char *> argv;
    for (auto & a : args) {
      argv.push_back(a.data());
    }
    argv.push_back(nullptr);
    pid_ = fork();
    if (pid_ == 0) {
      execvp(argv[0], argv.data());
      _exit(127);
    }
    for (int i = 0; i < 100; ++i) {
      struct stat st;
      if (stat(link_.c_str(), &st) == 0) {
        return;
      }
      std::this_thread::sleep_for(50ms);
    }
    FAIL() << "the fake bus didn't start";
  }

  // A second connection to the bus, for looking at the servos once the driver has closed its own.
  std::unique_ptr<StsBus> probe()
  {
    auto bus = std::make_unique<StsBus>();
    bus->open(link_, 1000000, 50);
    return bus;
  }

  void TearDown() override
  {
    if (pid_ > 0) {
      kill(pid_, SIGTERM);
      waitpid(pid_, nullptr, 0);
    }
  }

  // Runs read/write cycles at 100 Hz for `seconds`.
  static void run(FeetechStsSystem & hw, double seconds)
  {
    const rclcpp::Duration period(0, 10000000);
    const rclcpp::Time t0(0, 0, RCL_ROS_TIME);
    for (int i = 0; i < static_cast<int>(seconds * 100); ++i) {
      ASSERT_EQ(hw.read(t0, period), hardware_interface::return_type::OK);
      ASSERT_EQ(hw.write(t0, period), hardware_interface::return_type::OK);
      std::this_thread::sleep_for(10ms);
    }
  }

  std::string link_;
  pid_t pid_ = -1;
};

TEST_F(FakeBusTest, HoldsThenFollowsCommands)
{
  start_bus();
  auto infos = hardware_interface::parse_control_resources_from_urdf(urdf(link_));
  ASSERT_EQ(infos.size(), 1u);
  FeetechStsSystem hw;
  ASSERT_EQ(hw.on_init(infos[0]), CallbackReturn::SUCCESS);
  auto state_ifs = hw.export_state_interfaces();
  auto command_ifs = hw.export_command_interfaces();
  ASSERT_EQ(state_ifs.size(), 25u);
  ASSERT_EQ(command_ifs.size(), 5u);

  ASSERT_EQ(hw.on_configure(kUnconfigured), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw.on_activate(kInactive), CallbackReturn::SUCCESS);

  auto value = [&](const std::string & name) {return state(state_ifs, name);};
  // fake servos start in the middle (2048 ticks) = joint zero
  EXPECT_NEAR(value("shoulder_lift/position"), 0.0, 1e-9);
  EXPECT_NEAR(value("shoulder_lift/voltage"), 7.4, 1e-9);
  // activation copies the measured position into the command: nothing moves
  EXPECT_NEAR(command_ifs[1].get_value(), 0.0, 1e-9);

  // ask for +0.5 rad on the shoulder and -0.3 rad on the wrist roll
  command_ifs[1].set_value(0.5);
  command_ifs[4].set_value(-0.3);
  run(hw, 1.0);
  EXPECT_NEAR(value("shoulder_lift/position"), 0.5, 0.01);
  EXPECT_NEAR(value("wrist_roll/position"), -0.3, 0.01);
  EXPECT_NEAR(value("elbow_flex/position"), 0.0, 0.01);

  // past the calibrated range: stops at max_ticks (3000 = +1.46 rad)
  command_ifs[1].set_value(3.0);
  run(hw, 1.5);
  EXPECT_NEAR(value("shoulder_lift/position"), (3000 - 2048) * kRadPerTick, 0.01);

  ASSERT_EQ(hw.on_deactivate(kActive), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw.on_cleanup(kInactive), CallbackReturn::SUCCESS);
  // stopping leaves the servos holding the arm rather than dropping it
  EXPECT_EQ(probe()->read_u8(2, reg::kTorqueEnable).value_or(9), 1);
}

TEST_F(FakeBusTest, RateLimitsJumps)
{
  start_bus();
  auto infos = hardware_interface::parse_control_resources_from_urdf(urdf(link_));
  FeetechStsSystem hw;
  ASSERT_EQ(hw.on_init(infos[0]), CallbackReturn::SUCCESS);
  auto state_ifs = hw.export_state_interfaces();
  auto command_ifs = hw.export_command_interfaces();
  ASSERT_EQ(hw.on_configure(kUnconfigured), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw.on_activate(kInactive), CallbackReturn::SUCCESS);
  command_ifs[0].set_value(1.2);
  run(hw, 0.2);  // 20 cycles at max_velocity 3 rad/s -> at most 0.6 rad sent
  const double pan = state(state_ifs, "shoulder_pan/position");
  EXPECT_LT(pan, 0.65);
  EXPECT_GT(pan, 0.2);
  ASSERT_EQ(hw.on_deactivate(kActive), CallbackReturn::SUCCESS);
}

TEST_F(FakeBusTest, MissingServoFailsConfigure)
{
  start_bus();
  auto infos = hardware_interface::parse_control_resources_from_urdf(urdf(link_, "extra_joint"));
  FeetechStsSystem hw;
  ASSERT_EQ(hw.on_init(infos[0]), CallbackReturn::SUCCESS);
  EXPECT_EQ(hw.on_configure(kUnconfigured), CallbackReturn::ERROR);
}

TEST_F(FakeBusTest, TorqueOffOnlyReads)
{
  start_bus();
  probe()->write_u8(3, reg::kTorqueEnable, 1);  // still holding from the last run
  auto infos = hardware_interface::parse_control_resources_from_urdf(urdf(link_, "", "false"));
  FeetechStsSystem hw;
  ASSERT_EQ(hw.on_init(infos[0]), CallbackReturn::SUCCESS);
  auto command_ifs = hw.export_command_interfaces();
  ASSERT_EQ(hw.on_configure(kUnconfigured), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw.on_activate(kInactive), CallbackReturn::SUCCESS);
  command_ifs[2].set_value(0.8);
  run(hw, 0.3);
  hw.on_deactivate(kActive);
  hw.on_cleanup(kInactive);

  // the motors went off and nothing moved, and the EEPROM settings were applied once
  auto bus = probe();
  EXPECT_EQ(bus->read_u16(3, reg::kPresentPosition).value_or(0), 2048);
  EXPECT_EQ(bus->read_u8(3, reg::kTorqueEnable).value_or(9), 0);
  EXPECT_EQ(bus->read_u8(3, reg::kPCoefficient).value_or(0), 16);
  EXPECT_EQ(bus->read_u8(3, reg::kReturnDelayTime).value_or(9), 0);
}

TEST_F(FakeBusTest, ALittleOutsideItsRangeComesBackSlowly)
{
  start_bus({3030});  // shoulder_pan 30 ticks past its max_ticks, 3000
  auto infos = hardware_interface::parse_control_resources_from_urdf(urdf(link_));
  FeetechStsSystem hw;
  ASSERT_EQ(hw.on_init(infos[0]), CallbackReturn::SUCCESS);
  auto state_ifs = hw.export_state_interfaces();
  auto command_ifs = hw.export_command_interfaces();
  ASSERT_EQ(hw.on_configure(kUnconfigured), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw.on_activate(kInactive), CallbackReturn::SUCCESS);
  // the motor holds it where it is: no jump to the end of the range
  std::this_thread::sleep_for(100ms);
  ASSERT_EQ(hw.read(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration(0, 10000000)), return_type::OK);
  EXPECT_NEAR(state(state_ifs, "shoulder_pan/position"), (3030 - 2048) * kRadPerTick, 1e-9);
  // then the commands bring it back to the end of its range, at max_velocity
  run(hw, 0.5);
  EXPECT_NEAR(state(state_ifs, "shoulder_pan/position"), (3000 - 2048) * kRadPerTick, 0.005);
  ASSERT_EQ(hw.on_deactivate(kActive), CallbackReturn::SUCCESS);
}

TEST_F(FakeBusTest, FarOutsideItsRangeLeavesTheMotorsOff)
{
  start_bus({2048, 2048, 3500});  // elbow_flex 500 ticks past max_ticks: the calibration doesn't fit
  auto infos = hardware_interface::parse_control_resources_from_urdf(urdf(link_));
  FeetechStsSystem hw;
  ASSERT_EQ(hw.on_init(infos[0]), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw.on_configure(kUnconfigured), CallbackReturn::SUCCESS);
  EXPECT_EQ(hw.on_activate(kInactive), CallbackReturn::FAILURE);
  hw.on_cleanup(kInactive);
  auto bus = probe();
  EXPECT_EQ(bus->read_u8(3, reg::kTorqueEnable).value_or(9), 0);
  EXPECT_EQ(bus->read_u16(3, reg::kPresentPosition).value_or(0), 3500);
}

}  // namespace
