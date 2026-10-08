// The real driver, end to end, against the Python fake bus on a pseudo-terminal.
#include <gtest/gtest.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "so101_scan_hardware/arm_model.hpp"
#include "so101_scan_hardware/feetech_sts_system.hpp"
#include "so101_scan_hardware/sts_bus.hpp"
#include "so101_scan_interfaces/msg/arm_safety.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#ifndef PYTHON_EXECUTABLE
#define PYTHON_EXECUTABLE "python3"
#endif

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using so101_scan_hardware::ArmModel;
using so101_scan_hardware::FeetechStsSystem;
using so101_scan_hardware::SafetyState;
using so101_scan_hardware::StsBus;
namespace reg = so101_scan_hardware::sts::reg;
using namespace std::chrono_literals;

namespace
{

const rclcpp_lifecycle::State kUnconfigured(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
const rclcpp_lifecycle::State kInactive(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
const rclcpp_lifecycle::State kActive(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");

// What the test arm's URDF has besides the five servos.
struct Arm
{
  std::string extra_joint;   // a sixth joint, at id 9, which the fake bus doesn't have
  std::string torque = "true";
  std::string params;        // more <param> elements for the driver
  bool links = false;        // a kinematic chain with a head, so the soft limits run
};

// The chain for Arm::links: shoulder_pan (z) 0.1 m up, then shoulder_lift, elbow_flex and
// wrist_flex (y) with 0.1 m between them along x, wrist_roll (x) 0.02 m further, and a 20 mm,
// 1 kg head at the end, 0.22 m out. At zero the arm is level.
const char * kChain = R"(
  <link name="base_link"/><link name="shoulder_link"/><link name="upper_arm_link"/>
  <link name="lower_arm_link"/><link name="wrist_link"/>
  <link name="wrist_roll_link">
    <inertial><mass value="1.0"/><inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/></inertial>
  </link>
  <link name="scan_head_link">
    <collision><geometry><box size="0.02 0.02 0.02"/></geometry></collision>
  </link>
  <joint name="shoulder_pan" type="revolute"><parent link="base_link"/><child link="shoulder_link"/>
    <origin xyz="0 0 0.1"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
  <joint name="shoulder_lift" type="revolute"><parent link="shoulder_link"/><child link="upper_arm_link"/>
    <axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
  <joint name="elbow_flex" type="revolute"><parent link="upper_arm_link"/><child link="lower_arm_link"/>
    <origin xyz="0.1 0 0"/><axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
  <joint name="wrist_flex" type="revolute"><parent link="lower_arm_link"/><child link="wrist_link"/>
    <origin xyz="0.1 0 0"/><axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
  <joint name="wrist_roll" type="revolute"><parent link="wrist_link"/><child link="wrist_roll_link"/>
    <origin xyz="0.02 0 0"/><axis xyz="1 0 0"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
  <joint name="head_mount" type="fixed"><parent link="wrist_roll_link"/><child link="scan_head_link"/></joint>
)";

std::string urdf(const std::string & port, const Arm & arm = {})
{
  std::string joints;
  const std::vector<std::pair<std::string, int>> servos = {
    {"shoulder_pan", 1}, {"shoulder_lift", 2}, {"elbow_flex", 3}, {"wrist_flex", 4}, {"wrist_roll", 5}};
  auto joint = [](const std::string & name, int id) {
      return "<joint name='" + name + "'><param name='id'>" + std::to_string(id) +
             "</param><param name='zero_ticks'>2048</param><param name='sign'>1</param>"
             "<param name='min_ticks'>1000</param><param name='max_ticks'>3000</param>"
             "<command_interface name='position'/><state_interface name='position'/>"
             "<state_interface name='velocity'/><state_interface name='load'/>"
             "<state_interface name='voltage'/><state_interface name='temperature'/></joint>";
    };
  for (const auto & [name, id] : servos) {
    joints += joint(name, id);
  }
  if (!arm.extra_joint.empty()) {
    joints += joint(arm.extra_joint, 9);
  }
  return "<?xml version='1.0'?><robot name='t'>" + std::string(arm.links ? kChain : "") +
         "<ros2_control name='arm' type='system'><hardware>"
         "<plugin>so101_scan_hardware/FeetechStsSystem</plugin>"
         "<param name='port'>" + port + "</param><param name='torque'>" + arm.torque + "</param>"
         "<param name='max_velocity'>3.0</param><param name='p_coefficient'>16</param>"
         "<param name='return_delay_time'>0</param>"
         "<param name='soft_limits'>" + (arm.links ? "true" : "false") + "</param>" + arm.params +
         "</hardware>" + joints + "</ros2_control></robot>";
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

// The driver configured and active on the fake bus, with its interfaces.
struct Driver
{
  FeetechStsSystem hw;
  std::vector<hardware_interface::StateInterface> states;
  std::vector<hardware_interface::CommandInterface> commands;
  double value(const std::string & name) {return state(states, name);}
  SafetyState safety() {return hw.safety_status().state;}
};

class FakeBusTest : public ::testing::Test
{
protected:
  // Starts the fake bus with five servos; ticks are start positions from id 1 on (default 2048).
  void start_bus(const std::vector<int> & ticks = {})
  {
    link_ = "/tmp/so101_test_bus_" + std::to_string(getpid());
    control_ = link_ + ".ctl";
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

  // Changes a fake servo while the driver runs (see fake_bus.py), e.g. {"temperature": 66}.
  void control(int id, const std::string & fields)
  {
    const std::string msg = "{\"id\": " + std::to_string(id) + ", " + fields + "}";
    int fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, control_.c_str(), sizeof(addr.sun_path) - 1);
    ASSERT_EQ(sendto(fd, msg.data(), msg.size(), 0, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)),
      static_cast<ssize_t>(msg.size()));
    close(fd);
    std::this_thread::sleep_for(20ms);  // let the bus apply it
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

  // Runs read/write cycles at 100 Hz for `seconds`, or until `until` says stop.
  static void run(FeetechStsSystem & hw, double seconds, std::function<bool()> until = nullptr)
  {
    const rclcpp::Duration period(0, 10000000);
    const rclcpp::Time t0(0, 0, RCL_ROS_TIME);
    for (int i = 0; i < static_cast<int>(seconds * 100); ++i) {
      ASSERT_EQ(hw.read(t0, period), hardware_interface::return_type::OK);
      ASSERT_EQ(hw.write(t0, period), hardware_interface::return_type::OK);
      if (until && until()) {
        return;
      }
      std::this_thread::sleep_for(10ms);
    }
  }

  std::unique_ptr<Driver> start_driver(const Arm & arm = {})
  {
    auto d = std::make_unique<Driver>();
    auto infos = hardware_interface::parse_control_resources_from_urdf(urdf(link_, arm));
    EXPECT_EQ(d->hw.on_init(infos[0]), CallbackReturn::SUCCESS);
    d->states = d->hw.export_state_interfaces();
    d->commands = d->hw.export_command_interfaces();
    EXPECT_EQ(d->hw.on_configure(kUnconfigured), CallbackReturn::SUCCESS);
    EXPECT_EQ(d->hw.on_activate(kInactive), CallbackReturn::SUCCESS);
    return d;
  }

  std::string link_, control_;
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
  Arm arm;
  arm.extra_joint = "extra_joint";
  auto infos = hardware_interface::parse_control_resources_from_urdf(urdf(link_, arm));
  FeetechStsSystem hw;
  ASSERT_EQ(hw.on_init(infos[0]), CallbackReturn::SUCCESS);
  EXPECT_EQ(hw.on_configure(kUnconfigured), CallbackReturn::ERROR);
}

TEST_F(FakeBusTest, TorqueOffOnlyReads)
{
  start_bus();
  probe()->write_u8(3, reg::kTorqueEnable, 1);  // still holding from the last run
  Arm arm;
  arm.torque = "false";
  auto infos = hardware_interface::parse_control_resources_from_urdf(urdf(link_, arm));
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

// --- safety ------------------------------------------------------------------------------

constexpr double kTick = kRadPerTick;
const std::vector<std::string> kJoints = {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"};

bool contains(const std::string & text, const std::string & part)
{
  return text.find(part) != std::string::npos;
}

TEST_F(FakeBusTest, AStallHoldsWhereTheJointStopped)
{
  start_bus();
  auto d = start_driver();
  control(1, "\"obstacle\": [0, 2300]");  // something in the way of shoulder_pan at 2300 ticks
  d->commands[0].set_value(0.8);           // 2048 + 521 ticks: past it
  run(d->hw, 3.0, [&]() {return d->safety() != SafetyState::kOk;});
  auto s = d->hw.safety_status();
  ASSERT_EQ(s.state, SafetyState::kHolding) << s.reason;
  EXPECT_TRUE(contains(s.reason, "shoulder_pan stalled")) << s.reason;
  // it holds where it stopped, not at its goal, so it stops pushing
  run(d->hw, 0.2);
  EXPECT_NEAR(d->value("shoulder_pan/position"), (2300 - 2048) * kTick, 2 * kTick);
  EXPECT_LT(std::fabs(d->value("shoulder_pan/load")), 0.05);
  // and the controllers are ignored until a reset
  d->commands[0].set_value(-0.3);
  run(d->hw, 0.5);
  EXPECT_NEAR(d->value("shoulder_pan/position"), (2300 - 2048) * kTick, 2 * kTick);

  // reset: nothing moves until the command asks for where the arm is
  control(1, "\"obstacle\": null");
  auto [ok, message] = d->hw.reset();
  ASSERT_TRUE(ok) << message;
  EXPECT_EQ(d->safety(), SafetyState::kResuming);
  run(d->hw, 0.5);
  EXPECT_NEAR(d->value("shoulder_pan/position"), (2300 - 2048) * kTick, 2 * kTick);
  // a controller that starts again starts from the pose the reset left in its command
  EXPECT_NEAR(d->commands[0].get_value(), d->value("shoulder_pan/position"), 1e-9);
  run(d->hw, 0.1);
  EXPECT_EQ(d->safety(), SafetyState::kOk);
  d->commands[0].set_value(-0.3);
  run(d->hw, 1.0);
  EXPECT_NEAR(d->value("shoulder_pan/position"), -0.3, 0.01);
  ASSERT_EQ(d->hw.on_deactivate(kActive), CallbackReturn::SUCCESS);
}

TEST_F(FakeBusTest, AStaleCommandWaitsAfterAReset)
{
  start_bus();
  auto d = start_driver();
  auto [stopped, why] = d->hw.stop(false, "test e-stop");
  ASSERT_TRUE(stopped) << why;
  EXPECT_EQ(d->hw.safety_status().reason, "test e-stop");
  d->commands[1].set_value(0.6);  // a controller still running its trajectory
  run(d->hw, 0.5);
  EXPECT_NEAR(d->value("shoulder_lift/position"), 0.0, 2 * kTick);
  ASSERT_TRUE(d->hw.reset().first);
  d->commands[1].set_value(0.6);  // the old goal: ignored
  run(d->hw, 0.5);
  EXPECT_EQ(d->safety(), SafetyState::kResuming);
  EXPECT_NEAR(d->value("shoulder_lift/position"), 0.0, 2 * kTick);
  d->commands[1].set_value(0.02);  // within resume_tolerance of where it is: followed again
  run(d->hw, 0.5);
  EXPECT_EQ(d->safety(), SafetyState::kOk);
  EXPECT_NEAR(d->value("shoulder_lift/position"), 0.02, 2 * kTick);
  ASSERT_EQ(d->hw.on_deactivate(kActive), CallbackReturn::SUCCESS);
}

TEST_F(FakeBusTest, OverheatingHoldsThenSwitchesTheMotorsOff)
{
  start_bus();
  Arm arm;
  arm.params = "<param name='torque_off_after'>1.0</param>";
  auto d = start_driver(arm);
  control(3, "\"temperature\": 58");
  run(d->hw, 0.3);
  auto s = d->hw.safety_status();
  EXPECT_EQ(s.state, SafetyState::kOk);
  ASSERT_EQ(s.warnings.size(), 1u);
  EXPECT_TRUE(contains(s.warnings[0], "elbow_flex at 58 C (warning at 55 C)")) << s.warnings[0];
  control(3, "\"temperature\": 66");
  run(d->hw, 2.0, [&]() {return d->safety() == SafetyState::kHolding;});
  EXPECT_TRUE(contains(d->hw.safety_status().reason, "elbow_flex at 66 C (limit 65 C)"));
  // holding doesn't cool it: after torque_off_after the motors go off
  run(d->hw, 2.0, [&]() {return d->safety() == SafetyState::kTorqueOff;});
  s = d->hw.safety_status();
  ASSERT_EQ(s.state, SafetyState::kTorqueOff);
  EXPECT_TRUE(contains(s.reason, "; then still elbow_flex at 66 C (limit 65 C) after 1.0 s holding")) << s.reason;
  // no reset while it is hot
  auto [ok, message] = d->hw.reset();
  EXPECT_FALSE(ok);
  EXPECT_TRUE(contains(message, "still: elbow_flex at 66 C")) << message;
  control(3, "\"temperature\": 40");
  run(d->hw, 0.1);
  std::tie(ok, message) = d->hw.reset();
  ASSERT_TRUE(ok) << message;
  EXPECT_EQ(d->safety(), SafetyState::kResuming);
  ASSERT_EQ(d->hw.on_deactivate(kActive), CallbackReturn::SUCCESS);
  ASSERT_EQ(d->hw.on_cleanup(kInactive), CallbackReturn::SUCCESS);
  EXPECT_EQ(probe()->read_u8(3, reg::kTorqueEnable).value_or(9), 1);  // the reset switched it back on
}

TEST_F(FakeBusTest, OverloadAndTorqueOffEstop)
{
  start_bus();
  auto d = start_driver();
  control(2, "\"extra_load\": 950");  // the shoulder holding more than it should
  run(d->hw, 0.5);
  EXPECT_EQ(d->safety(), SafetyState::kOk);  // not for long enough yet
  run(d->hw, 1.5, [&]() {return d->safety() != SafetyState::kOk;});
  ASSERT_EQ(d->safety(), SafetyState::kHolding);
  EXPECT_TRUE(contains(d->hw.safety_status().reason, "shoulder_lift at 95% of its maximum torque"));
  // the torque-off e-stop on top of a hold
  ASSERT_TRUE(d->hw.stop(true, "limp").first);
  EXPECT_EQ(d->safety(), SafetyState::kTorqueOff);
  EXPECT_TRUE(contains(d->hw.safety_status().reason, "; then limp"));
  ASSERT_EQ(d->hw.on_deactivate(kActive), CallbackReturn::SUCCESS);
  ASSERT_EQ(d->hw.on_cleanup(kInactive), CallbackReturn::SUCCESS);
  EXPECT_EQ(probe()->read_u8(2, reg::kTorqueEnable).value_or(9), 0);
}

TEST_F(FakeBusTest, TorqueOffOnlyWarns)
{
  start_bus();
  Arm arm;
  arm.torque = "false";
  auto d = start_driver(arm);
  control(4, "\"temperature\": 70");
  run(d->hw, 1.5);
  auto s = d->hw.safety_status();
  EXPECT_EQ(s.state, SafetyState::kReadOnly);
  bool reported = false;
  for (const auto & w : s.warnings) {
    reported = reported || contains(w, "over the limit: wrist_flex at 70 C");
  }
  EXPECT_TRUE(reported);
  EXPECT_TRUE(d->hw.stop(false, "e-stop").first);  // nothing to stop
  EXPECT_EQ(d->safety(), SafetyState::kReadOnly);
}

// The soft limits, with the test chain: what the arm model says at the pose the arm stopped in.
std::vector<double> joints_of(Driver & d)
{
  std::vector<double> q;
  for (const auto & j : kJoints) {
    q.push_back(d.value(j + "/position"));
  }
  return q;
}

TEST_F(FakeBusTest, SoftLimitStopsTheHeadAboveTheTable)
{
  start_bus();
  Arm arm;
  arm.links = true;
  arm.params = "<param name='table_z'>0.0</param><param name='stall_torque'>100</param>";
  auto d = start_driver(arm);
  d->commands[1].set_value(1.0);  // tip the level arm 57 deg down: the head would go 85 mm into the table
  run(d->hw, 2.0);
  ArmModel model;
  model.load(urdf(link_, arm), kJoints, {"upper_arm_link", "lower_arm_link", "wrist_link", "wrist_roll_link"},
    "scan_head_link");
  so101_scan_hardware::WorkspaceLimits w;
  w.table_z = 0.0;
  const auto e = model.evaluate(joints_of(*d), w);
  // stopped at the edge: 10 mm above the table, give or take a cycle's step
  EXPECT_EQ(e.workspace_limit, "table");
  EXPECT_GT(e.workspace_margin, -0.002);
  EXPECT_LT(e.workspace_margin, 0.008);
  auto s = d->hw.safety_status();
  EXPECT_EQ(s.state, SafetyState::kOk);  // a soft limit isn't a stop
  ASSERT_FALSE(s.warnings.empty());
  EXPECT_TRUE(contains(s.warnings.back(), "soft limit: the head would come within")) << s.warnings.back();
  // back up is fine
  d->commands[1].set_value(0.0);
  run(d->hw, 1.0);
  EXPECT_NEAR(d->value("shoulder_lift/position"), 0.0, 2 * kTick);
  EXPECT_TRUE(d->hw.safety_status().warnings.empty());
  ASSERT_EQ(d->hw.on_deactivate(kActive), CallbackReturn::SUCCESS);
}

TEST_F(FakeBusTest, SoftLimitKeepsToTheGravityBudget)
{
  start_bus({2048, 1024});  // shoulder_lift at -90 deg: the arm points straight up, no gravity torque
  Arm arm;
  arm.links = true;
  // level, the 1 kg head 0.22 m out needs 2.16 N m: 60% of 3.6 N m, over the 50% limit
  arm.params = "<param name='stall_torque'>3.6</param><param name='max_gravity_load'>0.5</param>"
    "<param name='warn_gravity_load'>0.4</param>";
  auto d = start_driver(arm);
  d->commands[1].set_value(0.0);
  run(d->hw, 2.0);
  // stopped where sin(angle from vertical) * 0.6 = 0.5
  const double from_vertical = d->value("shoulder_lift/position") + M_PI / 2;
  EXPECT_NEAR(std::sin(from_vertical) * 0.6, 0.5, 0.02);
  auto s = d->hw.safety_status();
  EXPECT_NEAR(s.gravity_load[1], 0.5, 0.02);
  EXPECT_LE(s.gravity_load[1], 0.5);
  EXPECT_TRUE(contains(s.warnings.back(), "shoulder_lift would hold")) << s.warnings.back();
  ASSERT_EQ(d->hw.on_deactivate(kActive), CallbackReturn::SUCCESS);
}

// The same over ROS: the services, the /estop topic and the state topic.
TEST_F(FakeBusTest, SafetyServicesAndTopics)
{
  rclcpp::InitOptions options;
  options.set_domain_id(40 + getpid() % 20);
  rclcpp::init(0, nullptr, options);
  start_bus();
  auto d = start_driver();
  auto node = rclcpp::Node::make_shared("safety_test");
  so101_scan_interfaces::msg::ArmSafety last;
  bool got = false;
  auto sub = node->create_subscription<so101_scan_interfaces::msg::ArmSafety>(
    "/arm_safety/state", rclcpp::QoS(1).reliable().transient_local(),
    [&](const so101_scan_interfaces::msg::ArmSafety & m) {last = m; got = true;});
  auto estop_pub = node->create_publisher<std_msgs::msg::Bool>("/estop", 10);
  auto call = [&](const std::string & name) {
      auto client = node->create_client<std_srvs::srv::Trigger>(name);
      EXPECT_TRUE(client->wait_for_service(5s)) << name;
      auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
      // the control loop has to keep running while the service waits for the bus
      for (int i = 0; i < 300 && future.wait_for(0s) != std::future_status::ready; ++i) {
        run(d->hw, 0.01);
        rclcpp::spin_some(node);
      }
      return future.get();
    };
  auto wait_state = [&](uint8_t state) {
      for (int i = 0; i < 300 && !(got && last.state == state); ++i) {
        run(d->hw, 0.01);
        rclcpp::spin_some(node);
      }
      return got && last.state == state;
    };
  EXPECT_TRUE(wait_state(so101_scan_interfaces::msg::ArmSafety::OK));
  EXPECT_EQ(last.joint_names.size(), 5u);
  EXPECT_NEAR(last.voltage[0], 7.4, 1e-9);

  auto r = call("/arm_safety/estop");
  EXPECT_TRUE(r->success) << r->message;
  EXPECT_TRUE(wait_state(so101_scan_interfaces::msg::ArmSafety::HOLDING));
  EXPECT_EQ(last.reason, "e-stop (/arm_safety/estop)");
  r = call("/arm_safety/reset");
  EXPECT_TRUE(r->success) << r->message;
  EXPECT_TRUE(wait_state(so101_scan_interfaces::msg::ArmSafety::OK));  // the command is where the arm is

  std_msgs::msg::Bool stop;
  stop.data = true;
  for (int i = 0; i < 300 && d->safety() != SafetyState::kHolding; ++i) {
    estop_pub->publish(stop);
    run(d->hw, 0.01);
  }
  EXPECT_TRUE(wait_state(so101_scan_interfaces::msg::ArmSafety::HOLDING));
  EXPECT_EQ(last.reason, "e-stop (/estop)");
  r = call("/arm_safety/torque_off");
  EXPECT_TRUE(r->success) << r->message;
  EXPECT_TRUE(wait_state(so101_scan_interfaces::msg::ArmSafety::TORQUE_OFF));

  ASSERT_EQ(d->hw.on_deactivate(kActive), CallbackReturn::SUCCESS);
  ASSERT_EQ(d->hw.on_cleanup(kInactive), CallbackReturn::SUCCESS);
  d.reset();
  rclcpp::shutdown();
}

}  // namespace
