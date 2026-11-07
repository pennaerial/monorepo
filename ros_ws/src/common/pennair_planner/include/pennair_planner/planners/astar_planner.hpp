#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

#include "pennair_planner/path_planner.hpp"

namespace pennair_planner
{

class AStarPlanner : public PathPlanner
{
public:
  using OccupancyGrid = nav_msgs::msg::OccupancyGrid;

  struct Config {
    int occupied_threshold = 65;
    double unknown_cost = 3.0;
    double occupancy_cost_scale = 2.0;
    double inflation_radius = 0.35;
  };

  AStarPlanner();

  explicit AStarPlanner(const Config& config);

  void set_costmap(OccupancyGrid::ConstSharedPtr costmap) override;

  bool plan(const Pose& start_pose, const Pose& goal_pose, nav_msgs::msg::Path& path) override;

private:
  Config config_;
  OccupancyGrid::ConstSharedPtr costmap_;
};

}  // namespace pennair_planner