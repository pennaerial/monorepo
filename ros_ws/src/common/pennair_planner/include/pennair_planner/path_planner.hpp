#pragma once

#include <Eigen/Geometry>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <queue>
#include <utility>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace pennair_planner
{

/**
 * @brief Interface for path planning algorithms.
 *
 * The planner service only depends on this class, so a new algorithm is added by subclassing it
 * and constructing it in the service node.
 */
class PathPlanner
{
public:
  using Pose = geometry_msgs::msg::Pose;

  virtual ~PathPlanner() = default;

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

/**
 * @brief Placeholder planner: a straight line from start_pose to goal_pose.
 *
 * Orientation is interpolated by taking the rotation from start to goal as an axis and an angle,
 * and rotating about that fixed axis by a fraction of the angle at each waypoint.
 */
class StraightLinePlanner : public PathPlanner
{
public:
  /**
   * @brief Constructs the planner.
   *
   * @param num_steps Number of segments the line is split into; the path has num_steps + 1 poses.
   */
  explicit StraightLinePlanner(int num_steps = 10) : num_steps_(num_steps) {}

  /**
   * @brief Plans a straight line from start_pose to goal_pose with evenly spaced waypoints.
   *
   * @param start_pose Pose of the first waypoint.
   * @param goal_pose Pose of the last waypoint.
   * @param[in,out] path Path the waypoints are appended to. `path.header` must be set by the
   *   caller and is used for every waypoint.
   * @return Always true; a straight line can always be planned.
   */
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
  /**
   * @brief Converts a quaternion message to a unit Eigen quaternion.
   *
   * @param q Quaternion message, not necessarily normalized.
   * @return The normalized quaternion, or identity if q is all zeros (e.g. an unset field).
   */
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


/**
 * @brief Costmap-based A* path planner
 *
 * Searches an occupancy grid of surrounding 8 cells. Occupied cells are blocked,
 * unknown cells give a penalty, and obstacles are
 * expanded for rover clearance.
 */
class AStarPlanner : public PathPlanner
{
public:
  using OccupancyGrid = nav_msgs::msg::OccupancyGrid;

  struct Config
  {
    int occupied_threshold = 65;
    double unknown_cost = 3.0;
    double occupancy_cost_scale = 2.0;
    double inflation_radius = 0.35;
  };

  AStarPlanner() = default;

  explicit AStarPlanner(const Config& config) : config_(config) {}

  /**
   * @brief Updates the costmap
   *
   * Service node should make sure it has a costmap
   * before calling plan().
   */
  void set_costmap(OccupancyGrid::ConstSharedPtr costmap)
  {
    costmap_ = std::move(costmap);
  }

  /**
   * @brief Plans A* path in the grid.
   *
   * Start and end positions relative to costmap.
   *
   * @return true if a path was found, false otherwise.
   */
  bool plan(
      const Pose& start_pose,
      const Pose& goal_pose,
      nav_msgs::msg::Path& path
  ) override
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

    const size_t count =
        static_cast<size_t>(width) * static_cast<size_t>(height);

    if (costmap_->data.size() != count) {
      return false;
    }

    const auto& q = info.origin.orientation;
    const double norm = std::sqrt(
        q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
    );

    if (norm < 1e-9) {
      return false;
    }

    const double yaw = std::atan2(
        2.0 * (q.w*q.z + q.x*q.y),
        1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    );

    if (std::abs(yaw) > 1e-6 ||
        std::abs(q.x) > 1e-6 ||
        std::abs(q.y) > 1e-6) {
      return false;
    }

    /// Convert world coords to grid coords
    const auto to_cell = [&](double x, double y, int& cx, int& cy) {
      const double grid_x =
          (x - info.origin.position.x) / info.resolution;
      const double grid_y =
          (y - info.origin.position.y) / info.resolution;

      if (!std::isfinite(grid_x) || !std::isfinite(grid_y) ||
          grid_x < 0.0 || grid_y < 0.0 ||
          grid_x >= width || grid_y >= height) {
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

    const auto index = [&](int x, int y) {
      return y * width + x;
    };

    const auto in_bounds = [&](int x, int y) {
      return x >= 0 && x < width && y >= 0 && y < height;
    };

    /// Expanding blocked cells for clearance
    std::vector<bool> blocked(count, false);

    const int radius_cells = static_cast<int>(
        std::ceil(std::max(0.0, config_.inflation_radius) /
                  info.resolution)
    );

    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const int value = costmap_->data[index(x, y)];

        if (value < config_.occupied_threshold) {
          continue;
        }

        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
          for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            if (std::hypot(dx, dy) * info.resolution >
                config_.inflation_radius) {
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

    ///For each entry, we have total cost and index of the cell
    struct QueueEntry
    {
      double f;
      int cell;

      bool operator>(const QueueEntry& other) const
      {
        return f > other.f;
      }
    };

    std::priority_queue<
        QueueEntry,
        std::vector<QueueEntry>,
        std::greater<QueueEntry>
    > open;

    const double infinity =
        std::numeric_limits<double>::infinity();

    std::vector<double> g_score(count, infinity);
    std::vector<int> parent(count, -1);

    /// Calculates distance
    const auto heuristic = [&](int x, int y) {
      const double dx = std::abs(x - gx);
      const double dy = std::abs(y - gy);
      const double diagonal = std::min(dx, dy);

      return (dx + dy +
              (std::sqrt(2.0) - 2.0) * diagonal)
             * info.resolution;
    };

    ///Unknown cells more expensive (maybe obscured by plane, idk)
    const auto traversal_multiplier = [&](int cell) {
      const int value = costmap_->data[cell];

      if (value < 0) {
        return std::max(1.0, config_.unknown_cost);
      }

      return 1.0 +
          std::max(0.0, config_.occupancy_cost_scale) *
          static_cast<double>(value) /
          std::max(1, config_.occupied_threshold);
    };

    constexpr std::array<std::pair<int, int>, 8> directions{{
        {-1, 0}, {1, 0}, {0, -1}, {0, 1},
        {-1, -1}, {-1, 1}, {1, -1}, {1, 1}
    }};

    g_score[start] = 0.0;
    open.push({heuristic(sx, sy), start});

    bool found = false;

    // A* search loop.
    while (!open.empty()) {
      const QueueEntry current = open.top();
      open.pop();

      const int x = current.cell % width;
      const int y = current.cell / width;

      ///Discard outdated entries in the queue
      if (current.f > g_score[current.cell] +
                          heuristic(x, y) + 1e-9) {
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
          if (blocked[index(x + dx, y)] ||
              blocked[index(x, y + dy)]) {
            continue;
          }
        }

        const double distance =
            std::hypot(dx, dy) * info.resolution;

        const double tentative =
            g_score[current.cell] +
            distance * traversal_multiplier(next);

        if (tentative < g_score[next]) {
          g_score[next] = tentative;
          parent[next] = current.cell;

          open.push({
              tentative + heuristic(nx, ny),
              next
          });
        }
      }
    }

    if (!found) {
      return false;
    }

    /// Rebuild path
    std::vector<int> cells;

    for (int current = goal; current != -1;
         current = parent[current]) {
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

      waypoint.pose.position.x =
          info.origin.position.x +
          (x + 0.5) * info.resolution;

      waypoint.pose.position.y =
          info.origin.position.y +
          (y + 0.5) * info.resolution;

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

    path.poses.insert(
        path.poses.end(),
        waypoints.begin(),
        waypoints.end()
    );

    return true;
  }

private:
  Config config_;
  OccupancyGrid::ConstSharedPtr costmap_;
};


}  // namespace pennair_planner
