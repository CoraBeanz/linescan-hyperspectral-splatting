// The arm as its URDF describes it, for the driver's soft limits.
//
// Forward kinematics of the links from base_link down to the scan head, the static gravity
// torque on each driven joint from the links' masses, and how far a pose keeps the arm inside
// its workspace. The workspace, in base_link (metres):
//
//   * the table: the origins of the checked links and the eight corners of the head's
//     collision box stay table_clearance above table_z;
//   * the base: the head's corners stay outside a vertical cylinder of base_keepout_radius
//     around the shoulder_pan axis, up to base_keepout_top. It holds the base, the shoulder
//     servo and the electronics beside them.
//
// so101_scan_safety checks scan plans with the same model in Python, and both read the
// numbers from the <ros2_control> block of the URDF, so a plan that passes there is one the
// driver lets through.
#pragma once

#include <Eigen/Geometry>

#include <array>
#include <string>
#include <vector>

namespace so101_scan_hardware
{

struct WorkspaceLimits
{
  double table_z = -0.0024;           // m, the table top in base_link
  double table_clearance = 0.01;      // m kept between the table and the arm
  double base_keepout_radius = 0.08;  // m around the shoulder_pan axis
  double base_keepout_top = 0.14;     // m, the top of that cylinder in base_link
};

class ArmModel
{
public:
  // joints: the driven joints, in the order positions are passed in. links: the links whose
  // origins must stay above the table. box_link: the link whose collision box is the head.
  // Throws std::runtime_error saying what is missing.
  void load(
    const std::string & urdf, const std::vector<std::string> & joints,
    const std::vector<std::string> & links, const std::string & box_link);
  bool loaded() const {return !nodes_.empty();}

  struct Evaluation
  {
    double workspace_margin = 0.0;  // m inside the workspace; negative is outside
    std::string workspace_limit;    // the limit closest to being crossed: "table" or "base"
    std::string workspace_part;     // and which part of the arm: a link name, or "head"
    std::vector<double> torque;     // N m of gravity on each driven joint
  };
  Evaluation evaluate(const std::vector<double> & q, const WorkspaceLimits & limits) const;

  // Pose of a link in base_link at joint positions q.
  Eigen::Isometry3d pose(const std::string & link, const std::vector<double> & q) const;
  const std::vector<std::string> & joints() const {return joints_;}

private:
  struct Node
  {
    std::string name;
    int parent = -1;                 // index into nodes_
    Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();  // parent link to joint frame
    int type = 0;                    // 0 fixed, 1 revolute or continuous, 2 prismatic
    Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
    int q_index = -1;                // which driven joint moves it, or -1
    double mass = 0.0;
    Eigen::Vector3d com = Eigen::Vector3d::Zero();  // in the link frame
  };

  void forward(const std::vector<double> & q, std::vector<Eigen::Isometry3d> & out) const;
  int index(const std::string & link) const;

  std::vector<Node> nodes_;             // parents before children
  std::vector<std::string> joints_;
  std::vector<int> joint_child_;        // per driven joint: the node it moves
  std::vector<std::vector<int>> below_; // per driven joint: the nodes it carries
  std::vector<int> checked_links_;
  int box_node_ = -1;
  std::array<Eigen::Vector3d, 8> box_corners_;  // in box_link's frame
  int pan_joint_ = -1;                  // the joint whose axis the base cylinder is around
};

}  // namespace so101_scan_hardware
