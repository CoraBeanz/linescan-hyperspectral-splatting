// The servo limits and the arm model on their own: no bus, no ROS.
#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "so101_scan_hardware/arm_model.hpp"
#include "so101_scan_hardware/servo_checks.hpp"
#include "so101_scan_hardware/sts_protocol.hpp"

using so101_scan_hardware::ArmModel;
using so101_scan_hardware::ServoChecks;
using so101_scan_hardware::ServoLimits;
using so101_scan_hardware::ServoSample;
using so101_scan_hardware::WorkspaceLimits;

namespace
{

constexpr double kDt = 0.01;

ServoSample calm()
{
  ServoSample s;
  s.fresh = true;
  s.load = 0.1;
  s.voltage = 7.4;
  s.temperature = 30;
  return s;
}

// Runs `seconds` of cycles with the same sample; returns the last result.
ServoChecks::Result run(ServoChecks & c, const ServoSample & s, double seconds, bool driving = true)
{
  ServoChecks::Result r;
  for (int i = 0; i < static_cast<int>(std::round(seconds / kDt)); ++i) {
    r = c.update({s}, kDt, driving);
  }
  return r;
}

ServoChecks make_checks()
{
  ServoChecks c;
  c.configure(ServoLimits{}, {"shoulder_lift"});
  c.set_supply_voltage(7.4);
  return c;
}

TEST(ServoChecks, LoadMustLastBeforeItStopsTheArm)
{
  auto c = make_checks();
  auto s = calm();
  s.load = -0.95;  // either direction
  auto r = run(c, s, 0.5);
  EXPECT_TRUE(r.fault.empty());  // a spike while accelerating
  ASSERT_EQ(r.present.size(), 1u);
  r = run(c, s, 0.6);
  EXPECT_NE(r.fault.find("shoulder_lift at 95% of its maximum torque"), std::string::npos) << r.fault;

  c.clear();
  s.load = 0.7;
  r = run(c, s, 5.0);
  EXPECT_TRUE(r.fault.empty());
  ASSERT_EQ(r.warnings.size(), 1u);
  EXPECT_NE(r.warnings[0].find("warning at 60%"), std::string::npos) << r.warnings[0];
}

TEST(ServoChecks, TemperatureAndVoltage)
{
  auto c = make_checks();
  auto s = calm();
  s.temperature = 58;
  auto r = run(c, s, 2.0);
  EXPECT_TRUE(r.fault.empty());
  EXPECT_EQ(r.warnings.size(), 1u);
  s.temperature = 66;
  r = run(c, s, 1.1);
  EXPECT_NE(r.fault.find("66 C (limit 65 C)"), std::string::npos) << r.fault;

  // automatic voltage limits: 80% and 90% of the supply at rest
  c = make_checks();
  EXPECT_NEAR(c.min_voltage(), 5.92, 1e-9);
  s = calm();
  s.voltage = 6.5;
  r = run(c, s, 2.0);
  EXPECT_TRUE(r.fault.empty());
  EXPECT_EQ(r.warnings.size(), 1u);
  s.voltage = 5.5;
  r = run(c, s, 0.6);
  EXPECT_NE(r.fault.find("supply at 5.5 V"), std::string::npos) << r.fault;
}

TEST(ServoChecks, StallIsFarFromTheGoalAndNotMoving)
{
  auto c = make_checks();
  auto s = calm();
  s.goal = 0.5;
  s.position = 0.2;     // 0.3 rad short
  s.velocity = 1.0;     // but still on its way
  EXPECT_TRUE(run(c, s, 2.0).fault.empty());
  s.velocity = 0.0;
  EXPECT_TRUE(run(c, s, 2.0, false).fault.empty());  // motors off: no goal to fall short of
  auto r = run(c, s, 0.6);
  EXPECT_NE(r.fault.find("stalled 17.2 deg short"), std::string::npos) << r.fault;
  c.clear();
  s.position = 0.4;     // within stall_error of the goal: a sag, not a stall
  EXPECT_TRUE(run(c, s, 2.0).fault.empty());
}

TEST(ServoChecks, ServoAlarmsNeedTwoReads)
{
  auto c = make_checks();
  auto s = calm();
  s.error = so101_scan_hardware::sts::error::kOverload;
  EXPECT_TRUE(c.update({s}, kDt, true).fault.empty());
  auto r = c.update({s}, kDt, true);
  EXPECT_EQ(r.fault, "shoulder_lift reports overload");
  // the angle-limit bit is only reported
  c.clear();
  s.error = so101_scan_hardware::sts::error::kAngle;
  r = run(c, s, 1.0);
  EXPECT_TRUE(r.fault.empty());
  EXPECT_EQ(r.warnings.size(), 1u);
  // a servo that didn't answer this cycle isn't judged on stale numbers
  c.clear();
  s = calm();
  s.fresh = false;
  s.temperature = 90;
  EXPECT_TRUE(run(c, s, 2.0).fault.empty());
}

// A two-joint arm: pan about z, then lift about y 0.15 m up, then an arm 0.2 m long along x
// with a 1 kg link halfway and a 0.5 kg, 20 mm head box at its end.
const char * kArm = R"(<?xml version="1.0"?>
<robot name="t">
  <link name="base_link"/>
  <link name="turret"/>
  <link name="arm">
    <inertial><origin xyz="0.1 0 0"/><mass value="1.0"/>
      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/></inertial>
  </link>
  <link name="head">
    <inertial><origin xyz="0 0 0"/><mass value="0.5"/>
      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/></inertial>
    <collision><origin xyz="0 0 0"/><geometry><box size="0.02 0.02 0.02"/></geometry></collision>
  </link>
  <joint name="pan" type="revolute">
    <parent link="base_link"/><child link="turret"/><origin xyz="0.04 0 0.1"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
  <joint name="lift" type="revolute">
    <parent link="turret"/><child link="arm"/><origin xyz="0 0 0.05"/><axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
  <joint name="mount" type="fixed">
    <parent link="arm"/><child link="head"/><origin xyz="0.2 0 0"/>
  </joint>
</robot>)";

TEST(ArmModel, KinematicsGravityAndWorkspace)
{
  ArmModel m;
  m.load(kArm, {"pan", "lift"}, {"arm"}, "head");
  ASSERT_TRUE(m.loaded());
  WorkspaceLimits w;
  w.table_z = 0.0;
  w.table_clearance = 0.01;
  w.base_keepout_radius = 0.08;
  w.base_keepout_top = 0.12;

  // level: the head 0.24 m out, 0.15 m up
  EXPECT_TRUE(m.pose("head", {0, 0}).translation().isApprox(Eigen::Vector3d(0.24, 0, 0.15)));
  auto e = m.evaluate({0, 0}, w);
  // gravity about the lift axis: 1 kg at 0.1 m and 0.5 kg at 0.2 m; none about the vertical pan
  EXPECT_NEAR(e.torque[1], 9.81 * (1.0 * 0.1 + 0.5 * 0.2), 1e-9);
  EXPECT_NEAR(e.torque[0], 0.0, 1e-9);
  // nearest limit: the base cylinder, whose top (0.12 m) is below the head's lower face (0.14 m),
  // so the distance runs from the cylinder's rim to the head's inner lower corners
  const double inner = std::hypot(0.19, 0.01);  // those corners from the pan axis
  EXPECT_NEAR(e.workspace_margin, std::hypot(inner - 0.08, 0.02), 1e-9);
  EXPECT_EQ(e.workspace_limit, "base");
  EXPECT_EQ(e.workspace_part, "head");

  // panned half a turn, the head is over the far side
  EXPECT_TRUE(m.pose("head", {M_PI, 0}).translation().isApprox(Eigen::Vector3d(-0.16, 0, 0.15), 1e-9));

  // lift +90 deg about y points the arm straight down: no gravity torque, head into the table
  e = m.evaluate({0, M_PI / 2}, w);
  EXPECT_NEAR(e.torque[1], 0.0, 1e-9);
  EXPECT_TRUE(m.pose("head", {0, M_PI / 2}).translation().isApprox(Eigen::Vector3d(0.04, 0, -0.05), 1e-9));
  EXPECT_NEAR(e.workspace_margin, -0.05 - 0.01 - 0.01, 1e-9);  // its lower face, past the clearance
  EXPECT_EQ(e.workspace_limit, "table");

  // lift -30 deg raises it clear of everything
  EXPECT_GT(m.evaluate({0, -M_PI / 6}, w).workspace_margin, 0.05);

  // a cylinder that reaches over the head: inside it by as far as its deepest corner
  w.base_keepout_top = 0.3;
  w.base_keepout_radius = 0.3;
  e = m.evaluate({0, 0}, w);
  EXPECT_NEAR(e.workspace_margin, inner - 0.3, 1e-9);
  EXPECT_EQ(e.workspace_limit, "base");
}

TEST(ArmModel, MissingPartsAreErrors)
{
  ArmModel m;
  EXPECT_THROW(m.load(kArm, {"pan", "elbow"}, {}, "head"), std::runtime_error);
  EXPECT_THROW(m.load(kArm, {"pan"}, {"nope"}, "head"), std::runtime_error);
  EXPECT_THROW(m.load(kArm, {"pan"}, {}, "arm"), std::runtime_error);  // no box on it
  EXPECT_THROW(m.load("<robot", {"pan"}, {}, "head"), std::runtime_error);
  EXPECT_FALSE(m.loaded());
}

}  // namespace
