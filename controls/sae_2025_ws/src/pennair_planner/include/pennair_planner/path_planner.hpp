#pragma once

#include <Eigen/Geometry>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace pennair_planner
{

// Interface for path planning algorithms. The planner service only depends on this class,
// so a new algorithm is added by subclassing it and constructing it in the service node.
class PathPlanner
{
public:
  using Pose = geometry_msgs::msg::Pose;

  virtual ~PathPlanner() = default;

  // Plans a path from start_pose to goal_pose.
  // `path.header` must be set by the caller and is used for every pose in the path.
  // Returns true if a path was found, false otherwise.
  virtual bool plan(const Pose& start_pose, const Pose& goal_pose, nav_msgs::msg::Path& path) = 0;
};

// Placeholder planner: a straight line from start_pose to goal_pose.
// Orientation is interpolated by taking the rotation from start to goal as an axis and an angle,
// and rotating about that fixed axis by a fraction of the angle at each waypoint.
class StraightLinePlanner : public PathPlanner
{
public:
  explicit StraightLinePlanner(int num_steps = 10) : num_steps_(num_steps) {}

  bool plan(const Pose& start_pose, const Pose& goal_pose, nav_msgs::msg::Path& path) override
  {
    const auto& start = start_pose.position;
    const auto& goal = goal_pose.position;

    // Relative rotation from start to goal (q_start^-1 * q_goal) as an axis and an angle.
    // Eigen picks the angle <= pi (the short way around), and a zero angle when there is no rotation.
    const Eigen::Quaterniond q_start = to_eigen(start_pose.orientation);
    const Eigen::Quaterniond q_goal = to_eigen(goal_pose.orientation);
    const Eigen::AngleAxisd rotation(q_start.conjugate() * q_goal);

    for (int i = 0; i <= num_steps_; ++i) {
      const double t = static_cast<double>(i) / num_steps_;

      geometry_msgs::msg::PoseStamped waypoint;
      waypoint.header = path.header;
      waypoint.pose.position.x = start.x + t * (goal.x - start.x);
      waypoint.pose.position.y = start.y + t * (goal.y - start.y);
      waypoint.pose.position.z = start.z + t * (goal.z - start.z);

      // Rotate from the start orientation by a fraction t of the angle about the axis
      const Eigen::Quaterniond q = q_start * Eigen::AngleAxisd(t * rotation.angle(), rotation.axis());
      waypoint.pose.orientation.x = q.x();
      waypoint.pose.orientation.y = q.y();
      waypoint.pose.orientation.z = q.z();
      waypoint.pose.orientation.w = q.w();

      path.poses.push_back(waypoint);
    }
    return true;
  }

private:
  // Returns a unit quaternion. An all-zero quaternion (e.g. an unset field) becomes identity.
  static Eigen::Quaterniond to_eigen(const geometry_msgs::msg::Quaternion& q)
  {
    const Eigen::Quaterniond out(q.w, q.x, q.y, q.z);
    if (out.norm() < 1e-9) {
      return Eigen::Quaterniond::Identity();
    }
    return out.normalized();
  }

  int num_steps_;
};

}  // namespace pennair_planner
