#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"

namespace pennair_planner
{

/**
 * @brief Interface for path planning algorithms.
 *
 * The planner service only depends on this class, so new algorithms
 * can be added as plugins without modifying the service.
 */
class PathPlanner
{
public:
  using Pose = geometry_msgs::msg::Pose;

  virtual ~PathPlanner() = default;

  virtual void set_costmap(nav_msgs::msg::OccupancyGrid::ConstSharedPtr costmap)
  {
    (void)costmap;
  }

  /**
   * @brief Plans a path from start_pose to goal_pose.
   *
   * @param start_pose Pose the path starts at.
   * @param goal_pose Pose the path ends at.
   * @param[in,out] path Path the planned poses are appended to. `path.header` must be set by the
   *   caller and is used for every pose in the path.
   * @return true if a path was found, false otherwise.
   */
  virtual bool plan(const Pose& start_pose, const Pose& goal_pose, nav_msgs::msg::Path& path) = 0;
};
}  // namespace pennair_planner
