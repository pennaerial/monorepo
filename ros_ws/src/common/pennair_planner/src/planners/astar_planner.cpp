#include "pennair_planner/planners/astar_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <pluginlib/class_list_macros.hpp>
#include <queue>
#include <utility>
#include <vector>

namespace pennair_planner
{

AStarPlanner::AStarPlanner() = default;

AStarPlanner::AStarPlanner(const Config& config) : config_(config) {}

void AStarPlanner::set_costmap(OccupancyGrid::ConstSharedPtr costmap)
{
  costmap_ = std::move(costmap);
}

bool AStarPlanner::plan(const Pose& start_pose, const Pose& goal_pose, nav_msgs::msg::Path& path)
{
  if (!costmap_) {
    return false;
  }

  const auto& info = costmap_->info;
  const int width = static_cast<int>(info.width);
  const int height = static_cast<int>(info.height);

  if (width <= 0 || height <= 0 || info.resolution <= 0.0) {
    return false;
  }

  const size_t count = static_cast<size_t>(width) * static_cast<size_t>(height);

  if (costmap_->data.size() != count) {
    return false;
  }

  const auto& q = info.origin.orientation;
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);

  if (norm < 1e-9) {
    return false;
  }

  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  if (std::abs(yaw) > 1e-6 || std::abs(q.x) > 1e-6 || std::abs(q.y) > 1e-6) {
    return false;
  }

  /// Convert world coords to grid coords
  const auto to_cell = [&](double x, double y, int& cx, int& cy) {
    const double grid_x = (x - info.origin.position.x) / info.resolution;
    const double grid_y = (y - info.origin.position.y) / info.resolution;

    if (!std::isfinite(grid_x) || !std::isfinite(grid_y) || grid_x < 0.0 || grid_y < 0.0 || grid_x >= width ||
        grid_y >= height) {
      return false;
    }

    cx = static_cast<int>(std::floor(grid_x));
    cy = static_cast<int>(std::floor(grid_y));
    return true;
  };

  int sx, sy, gx, gy;

  if (!to_cell(start_pose.position.x, start_pose.position.y, sx, sy) ||
      !to_cell(goal_pose.position.x, goal_pose.position.y, gx, gy)) {
    return false;
  }

  const auto index = [&](int x, int y) { return y * width + x; };

  const auto in_bounds = [&](int x, int y) { return x >= 0 && x < width && y >= 0 && y < height; };

  /// Expanding blocked cells for clearance
  std::vector<bool> blocked(count, false);

  const int radius_cells = static_cast<int>(std::ceil(std::max(0.0, config_.inflation_radius) / info.resolution));

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const int value = costmap_->data[index(x, y)];

      if (value < config_.occupied_threshold) {
        continue;
      }

      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
          if (std::hypot(dx, dy) * info.resolution > config_.inflation_radius) {
            continue;
          }

          const int nx = x + dx;
          const int ny = y + dy;

          if (in_bounds(nx, ny)) {
            blocked[index(nx, ny)] = true;
          }
        }
      }
    }
  }

  const int start = index(sx, sy);
  const int goal = index(gx, gy);

  if (blocked[start] || blocked[goal]) {
    return false;
  }

  /// For each entry, we have total cost and index of the cell
  struct QueueEntry {
    double f;
    int cell;

    bool operator>(const QueueEntry& other) const
    {
      return f > other.f;
    }
  };

  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry> > open;

  const double infinity = std::numeric_limits<double>::infinity();

  std::vector<double> g_score(count, infinity);
  std::vector<int> parent(count, -1);

  /// Calculates distance
  const auto heuristic = [&](int x, int y) {
    const double dx = std::abs(x - gx);
    const double dy = std::abs(y - gy);
    const double diagonal = std::min(dx, dy);

    return (dx + dy + (std::sqrt(2.0) - 2.0) * diagonal) * info.resolution;
  };

  /// Unknown cells more expensive (maybe obscured by plane, idk)
  const auto traversal_multiplier = [&](int cell) {
    const int value = costmap_->data[cell];

    if (value < 0) {
      return std::max(1.0, config_.unknown_cost);
    }

    return 1.0 + std::max(0.0, config_.occupancy_cost_scale) * static_cast<double>(value) /
                     std::max(1, config_.occupied_threshold);
  };

  constexpr std::array<std::pair<int, int>, 8> directions{
      {{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}}
  };

  g_score[start] = 0.0;
  open.push({heuristic(sx, sy), start});

  bool found = false;

  // A* search loop.
  while (!open.empty()) {
    const QueueEntry current = open.top();
    open.pop();

    const int x = current.cell % width;
    const int y = current.cell / width;

    /// Discard outdated entries in the queue
    if (current.f > g_score[current.cell] + heuristic(x, y) + 1e-9) {
      continue;
    }

    if (current.cell == goal) {
      found = true;
      break;
    }

    for (const auto& direction : directions) {
      const int dx = direction.first;
      const int dy = direction.second;

      const int nx = x + dx;
      const int ny = y + dy;

      if (!in_bounds(nx, ny)) {
        continue;
      }

      const int next = index(nx, ny);

      if (blocked[next]) {
        continue;
      }

      /// Prevent diagonal movement through blocked corners
      if (dx != 0 && dy != 0) {
        if (blocked[index(x + dx, y)] || blocked[index(x, y + dy)]) {
          continue;
        }
      }

      const double distance = std::hypot(dx, dy) * info.resolution;

      const double tentative = g_score[current.cell] + distance * traversal_multiplier(next);

      if (tentative < g_score[next]) {
        g_score[next] = tentative;
        parent[next] = current.cell;

        open.push({tentative + heuristic(nx, ny), next});
      }
    }
  }

  if (!found) {
    return false;
  }

  /// Rebuild path
  std::vector<int> cells;

  for (int current = goal; current != -1; current = parent[current]) {
    cells.push_back(current);
  }

  std::reverse(cells.begin(), cells.end());

  std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  waypoints.reserve(cells.size() + 1);

  /// Convert grid back into world coordinates
  for (const int cell : cells) {
    const int x = cell % width;
    const int y = cell / width;

    geometry_msgs::msg::PoseStamped waypoint;
    waypoint.header = path.header;

    waypoint.pose.position.x = info.origin.position.x + (x + 0.5) * info.resolution;

    waypoint.pose.position.y = info.origin.position.y + (y + 0.5) * info.resolution;

    waypoint.pose.position.z = start_pose.position.z;
    waypoint.pose.orientation.w = 1.0;

    waypoints.push_back(waypoint);
  }

  /// Exact start and end coordinates
  waypoints.front().pose = start_pose;

  if (cells.size() == 1) {
    geometry_msgs::msg::PoseStamped goal_waypoint;
    goal_waypoint.header = path.header;
    goal_waypoint.pose = goal_pose;
    waypoints.push_back(goal_waypoint);
  } else {
    waypoints.back().pose.position = goal_pose.position;
  }

  /// Make rover turn towards next waypoint between waypoints
  for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
    const auto& a = waypoints[i].pose.position;
    const auto& b = waypoints[i + 1].pose.position;

    const double angle = std::atan2(b.y - a.y, b.x - a.x);

    waypoints[i].pose.orientation.x = 0.0;
    waypoints[i].pose.orientation.y = 0.0;
    waypoints[i].pose.orientation.z = std::sin(angle / 2.0);
    waypoints[i].pose.orientation.w = std::cos(angle / 2.0);
  }

  /// Rover ends with desired orientation
  waypoints.back().pose.orientation = goal_pose.orientation;

  path.poses.insert(path.poses.end(), waypoints.begin(), waypoints.end());

  return true;
}

}  // namespace pennair_planner

PLUGINLIB_EXPORT_CLASS(pennair_planner::AStarPlanner, pennair_planner::PathPlanner)