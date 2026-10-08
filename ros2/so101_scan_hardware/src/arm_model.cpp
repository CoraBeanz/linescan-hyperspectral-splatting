#include "so101_scan_hardware/arm_model.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <stdexcept>

#include "urdf/model.h"

namespace so101_scan_hardware
{
namespace
{

constexpr double kGravity = 9.81;

Eigen::Isometry3d to_eigen(const urdf::Pose & p)
{
  double x, y, z, w;
  p.rotation.getQuaternion(x, y, z, w);
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear() = Eigen::Quaterniond(w, x, y, z).normalized().toRotationMatrix();
  t.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  return t;
}

}  // namespace

void ArmModel::load(
  const std::string & urdf_xml, const std::vector<std::string> & joints,
  const std::vector<std::string> & links, const std::string & box_link)
{
  nodes_.clear();
  urdf::Model model;
  if (!model.initString(urdf_xml)) {
    throw std::runtime_error("can't parse the URDF");
  }
  joints_ = joints;

  // Every link, parents before children, with the joint that carries it.
  std::vector<std::string> joint_of;  // per node: the name of its parent joint
  std::deque<std::pair<urdf::LinkConstSharedPtr, int>> todo = {{model.getRoot(), -1}};
  while (!todo.empty()) {
    auto [link, parent] = todo.front();
    todo.pop_front();
    Node n;
    n.name = link->name;
    n.parent = parent;
    std::string joint_name;
    if (auto j = link->parent_joint) {
      joint_name = j->name;
      n.origin = to_eigen(j->parent_to_joint_origin_transform);
      n.axis = Eigen::Vector3d(j->axis.x, j->axis.y, j->axis.z);
      if (n.axis.norm() > 0) {
        n.axis.normalize();
      }
      if (j->type == urdf::Joint::REVOLUTE || j->type == urdf::Joint::CONTINUOUS) {
        n.type = 1;
      } else if (j->type == urdf::Joint::PRISMATIC) {
        n.type = 2;
      }
      auto it = std::find(joints.begin(), joints.end(), j->name);
      if (it != joints.end()) {
        n.q_index = static_cast<int>(it - joints.begin());
      }
    }
    if (link->inertial) {
      n.mass = link->inertial->mass;
      const auto & c = link->inertial->origin.position;
      n.com = Eigen::Vector3d(c.x, c.y, c.z);
    }
    const int self = static_cast<int>(nodes_.size());
    nodes_.push_back(n);
    joint_of.push_back(joint_name);
    for (const auto & child : link->child_links) {
      todo.push_back({child, self});
    }
  }

  joint_child_.assign(joints.size(), -1);
  for (std::size_t i = 0; i < nodes_.size(); ++i) {
    if (nodes_[i].q_index >= 0) {
      joint_child_[nodes_[i].q_index] = static_cast<int>(i);
    }
  }
  for (std::size_t k = 0; k < joints.size(); ++k) {
    if (joint_child_[k] < 0) {
      nodes_.clear();
      throw std::runtime_error("no joint '" + joints[k] + "' in the URDF");
    }
  }
  // A node is below joint k when its chain up to the root passes joint k's child.
  below_.assign(joints.size(), {});
  for (std::size_t i = 0; i < nodes_.size(); ++i) {
    for (int a = static_cast<int>(i); a >= 0; a = nodes_[a].parent) {
      if (nodes_[a].q_index >= 0) {
        below_[nodes_[a].q_index].push_back(static_cast<int>(i));
      }
    }
  }

  checked_links_.clear();
  for (const auto & name : links) {
    const int i = index(name);
    if (i < 0) {
      nodes_.clear();
      throw std::runtime_error("no link '" + name + "' in the URDF");
    }
    checked_links_.push_back(i);
  }
  box_node_ = index(box_link);
  auto box_urdf = model.getLink(box_link);
  urdf::CollisionSharedPtr box_collision;
  if (box_urdf) {
    for (const auto & c : box_urdf->collision_array) {
      if (c && c->geometry && c->geometry->type == urdf::Geometry::BOX) {
        box_collision = c;
        break;
      }
    }
  }
  if (box_node_ < 0 || !box_collision) {
    nodes_.clear();
    throw std::runtime_error("link '" + box_link + "' needs a box collision geometry");
  }
  const auto & dim = std::static_pointer_cast<urdf::Box>(box_collision->geometry)->dim;
  const Eigen::Isometry3d box_origin = to_eigen(box_collision->origin);
  int c = 0;
  for (int sx : {-1, 1}) {
    for (int sy : {-1, 1}) {
      for (int sz : {-1, 1}) {
        box_corners_[c++] = box_origin * Eigen::Vector3d(sx * dim.x / 2, sy * dim.y / 2, sz * dim.z / 2);
      }
    }
  }

  auto pan = std::find(joints.begin(), joints.end(), "shoulder_pan");
  pan_joint_ = pan == joints.end() ? 0 : static_cast<int>(pan - joints.begin());
}

int ArmModel::index(const std::string & link) const
{
  for (std::size_t i = 0; i < nodes_.size(); ++i) {
    if (nodes_[i].name == link) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

void ArmModel::forward(const std::vector<double> & q, std::vector<Eigen::Isometry3d> & out) const
{
  out.resize(nodes_.size());
  for (std::size_t i = 0; i < nodes_.size(); ++i) {
    const auto & n = nodes_[i];
    Eigen::Isometry3d t = n.parent < 0 ? Eigen::Isometry3d::Identity() : out[n.parent] * n.origin;
    const double v = n.q_index >= 0 && n.q_index < static_cast<int>(q.size()) ? q[n.q_index] : 0.0;
    if (n.type == 1) {
      t.rotate(Eigen::AngleAxisd(v, n.axis));
    } else if (n.type == 2) {
      t.translate(n.axis * v);
    }
    out[i] = t;
  }
}

Eigen::Isometry3d ArmModel::pose(const std::string & link, const std::vector<double> & q) const
{
  std::vector<Eigen::Isometry3d> t;
  forward(q, t);
  const int i = index(link);
  if (i < 0) {
    throw std::runtime_error("no link '" + link + "'");
  }
  return t[i];
}

ArmModel::Evaluation ArmModel::evaluate(const std::vector<double> & q, const WorkspaceLimits & limits) const
{
  Evaluation e;
  e.workspace_margin = std::numeric_limits<double>::infinity();
  if (nodes_.empty()) {
    return e;
  }
  std::vector<Eigen::Isometry3d> t;
  forward(q, t);
  auto consider = [&](double margin, const char * limit, const std::string & part) {
      if (margin < e.workspace_margin) {
        e.workspace_margin = margin;
        e.workspace_limit = limit;
        e.workspace_part = part;
      }
    };

  const double floor = limits.table_z + limits.table_clearance;
  for (int i : checked_links_) {
    consider(t[i].translation().z() - floor, "table", nodes_[i].name);
  }
  // The base cylinder is around the pan joint's axis, which is vertical in base_link.
  const auto & pan = nodes_[joint_child_[pan_joint_]];
  const Eigen::Vector3d axis_at = (pan.parent < 0 ? Eigen::Isometry3d::Identity() : t[pan.parent]) *
    pan.origin.translation();
  for (const auto & corner : box_corners_) {
    const Eigen::Vector3d w = t[box_node_] * corner;
    consider(w.z() - floor, "table", "head");
    // signed distance from the cylinder in (radius, height): positive outside it
    const double dr = std::hypot(w.x() - axis_at.x(), w.y() - axis_at.y()) - limits.base_keepout_radius;
    const double dz = w.z() - limits.base_keepout_top;
    consider(dr > 0 && dz > 0 ? std::hypot(dr, dz) : std::max(dr, dz), "base", "head");
  }

  // Gravity: the torque about each joint's axis from the weight of every link it carries.
  const Eigen::Vector3d g(0.0, 0.0, -kGravity);
  e.torque.assign(joints_.size(), 0.0);
  for (std::size_t k = 0; k < joints_.size(); ++k) {
    const auto & n = nodes_[joint_child_[k]];
    const Eigen::Isometry3d frame = (n.parent < 0 ? Eigen::Isometry3d::Identity() : t[n.parent]) * n.origin;
    const Eigen::Vector3d axis = frame.linear() * n.axis;
    double tau = 0.0;
    for (int i : below_[k]) {
      if (nodes_[i].mass > 0) {
        const Eigen::Vector3d r = t[i] * nodes_[i].com - frame.translation();
        tau += r.cross(nodes_[i].mass * g).dot(axis);
      }
    }
    e.torque[k] = tau;
  }
  return e;
}

}  // namespace so101_scan_hardware
