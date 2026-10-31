#pragma once

#include <Eigen/Geometry>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/path.hpp>

#include "pennair_planner/path_planner.hpp"

namespace pennair_planner
{

class StraightLinePlanner : public PathPlanner
{
public:
  StraightLinePlanner();
  explicit StraightLinePlanner(int num_steps);

  bool plan(const Pose& start_pose, const Pose& goal_pose, nav_msgs::msg::Path& path) override;

private:
  static Eigen::Quaterniond to_eigen(const geometry_msgs::msg::Quaternion& q);

  int num_steps_;
};

}  // namespace pennair_planner