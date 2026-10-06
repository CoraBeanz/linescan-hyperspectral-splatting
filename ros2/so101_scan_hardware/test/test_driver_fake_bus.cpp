// The real driver, end to end, against the Python fake bus on a pseudo-terminal.
#include <gtest/gtest.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
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
using so101_scan_hardware::FeetechStsSystem;
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

class FakeBusTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    link_ = "/tmp/so101_test_bus_" + std::to_string(getpid());
    pid_ = fork();
    if (pid_ == 0) {
      execlp(PYTHON_EXECUTABLE, PYTHON_EXECUTABLE, "-m", "so101_scan_hardware.fake_bus", "--link", link_.c_str(),
        static_cast<char *>(nullptr));
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

  auto value = [&](const std::string & name) -> double {
      for (auto & s : state_ifs) {
        if (s.get_name() == name) {
          return s.get_value();
        }
      }
      ADD_FAILURE() << "no state interface " << name;
      return NAN;
    };
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
  EXPECT_NEAR(value("shoulder_lift/position"), (3000 - 2048) * 2 * M_PI / 4096, 0.01);

  ASSERT_EQ(hw.on_deactivate(kActive), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw.on_cleanup(kInactive), CallbackReturn::SUCCESS);
}

TEST_F(FakeBusTest, RateLimitsJumps)
{
  auto infos = hardware_interface::parse_control_resources_from_urdf(urdf(link_));
  FeetechStsSystem hw;
  ASSERT_EQ(hw.on_init(infos[0]), CallbackReturn::SUCCESS);
  auto state_ifs = hw.export_state_interfaces();
  auto command_ifs = hw.export_command_interfaces();
  ASSERT_EQ(hw.on_configure(kUnconfigured), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw.on_activate(kInactive), CallbackReturn::SUCCESS);
  command_ifs[0].set_value(1.2);
  run(hw, 0.2);  // 20 cycles at max_velocity 3 rad/s -> at most 0.6 rad sent
  double pan = NAN;
  for (auto & s : state_ifs) {
    if (s.get_name() == "shoulder_pan/position") {
      pan = s.get_value();
    }
  }
  EXPECT_LT(pan, 0.65);
  EXPECT_GT(pan, 0.2);
  ASSERT_EQ(hw.on_deactivate(kActive), CallbackReturn::SUCCESS);
}

TEST_F(FakeBusTest, MissingServoFailsConfigure)
{
  auto infos = hardware_interface::parse_control_resources_from_urdf(urdf(link_, "extra_joint"));
  FeetechStsSystem hw;
  ASSERT_EQ(hw.on_init(infos[0]), CallbackReturn::SUCCESS);
  EXPECT_EQ(hw.on_configure(kUnconfigured), CallbackReturn::ERROR);
}

TEST_F(FakeBusTest, TorqueOffOnlyReads)
{
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

  // nothing moved, and the EEPROM settings were applied once
  so101_scan_hardware::StsBus bus;
  bus.open(link_, 1000000, 50);
  EXPECT_EQ(bus.read_u16(3, so101_scan_hardware::sts::reg::kPresentPosition).value_or(0), 2048);
  EXPECT_EQ(bus.read_u8(3, so101_scan_hardware::sts::reg::kTorqueEnable).value_or(9), 0);
  EXPECT_EQ(bus.read_u8(3, so101_scan_hardware::sts::reg::kPCoefficient).value_or(0), 16);
  EXPECT_EQ(bus.read_u8(3, so101_scan_hardware::sts::reg::kReturnDelayTime).value_or(9), 0);
}

}  // namespace
