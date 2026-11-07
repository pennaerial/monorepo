#include "pennair_planner/planners/straight_line_planner.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace pennair_planner
{

StraightLinePlanner::StraightLinePlanner() : num_steps_(10) {}

StraightLinePlanner::StraightLinePlanner(int num_steps) : num_steps_(num_steps) {}

bool StraightLinePlanner::plan(const Pose& start_pose, const Pose& goal_pose, nav_msgs::msg::Path& path)
{
  const auto& start = start_pose.position;
  const auto& goal = goal_pose.position;

  // Relative rotation from start to goal.
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

    const Eigen::Quaterniond q = q_start * Eigen::AngleAxisd(t * rotation.angle(), rotation.axis());

    waypoint.pose.orientation.x = q.x();
    waypoint.pose.orientation.y = q.y();
    waypoint.pose.orientation.z = q.z();
    waypoint.pose.orientation.w = q.w();

    path.poses.push_back(waypoint);
  }

  return true;
}

Eigen::Quaterniond StraightLinePlanner::to_eigen(const geometry_msgs::msg::Quaternion& q)
{
  const Eigen::Quaterniond out(q.w, q.x, q.y, q.z);

  if (out.norm() < 1e-9) {
    return Eigen::Quaterniond::Identity();
  }

  return out.normalized();
}

}  // namespace pennair_planner

PLUGINLIB_EXPORT_CLASS(pennair_planner::StraightLinePlanner, pennair_planner::PathPlanner)