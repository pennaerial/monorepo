#include <functional>
#include <memory>

#include "nav_msgs/msg/path.hpp"
#include "pennair_planner/path_planner.hpp"
#include "pennair_planner/srv/plan_path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <mutex>
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace pennair_planner
{

/**
 * @brief Node that serves path planning requests on the `plan_path` service.
 *
 * PlanPath.srv:
 *   - Request:  geometry_msgs/Pose start_pose, geometry_msgs/Pose goal_pose
 *   - Response: nav_msgs/Path path
 */
class PlannerService : public rclcpp::Node
{
public:
  using PlanPath = srv::PlanPath;

  /**
   * @brief Constructs the node and advertises the `plan_path` service.
   *
   * @param options Node options, passed in by the component container when loaded as a component.
   */
  explicit PlannerService(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("planner_service", options),
        planner_(std::make_unique<AStarPlanner>())  // swap in a different PathPlanner here
  {
    this->service_ = this->create_service<PlanPath>(
        "plan_path", std::bind(&PlannerService::handle_plan_path, this, std::placeholders::_1, std::placeholders::_2)
    );

    /// Creating publisher for path
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "planned_path", rclcpp::QoS(10).transient_local()

        /// Copy this line and customize endpoints to change displayed path
        /*ros2 service call /plan_path pennair_planner/srv/PlanPath \
        "{start_pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}, goal_pose: {position: {x: 5.0,
        y: 3.0, z: 0.0}, orientation: {w: 1.0}}}"*/

        /* ros2 service call /plan_path pennair_planner/srv/PlanPath \
"{start_pose: {position: {x: 1.25, y: 1.25, z: 0.0}, orientation: {w: 1.0}}, goal_pose: {position: {x: 4.25, y: 1.25, z: 0.0}, orientation: {w: 1.0}}}" */
    );

    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "costmap",
      rclcpp::QoS(1).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(costmap_mutex_);
        latest_costmap_ = msg;
      }
    );
  }

private:
  rclcpp::Service<PlanPath>::SharedPtr service_;
  std::unique_ptr<PathPlanner> planner_;
  // Publisher pointer so the path can actually be sent to RViz
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_costmap_;
  std::mutex costmap_mutex_;

  /**
   * @brief Runs the planner and fills in the response.
   *
   * This blocks the executor until planning is done.
   *
   * @param request Start and goal poses to plan between.
   * @param[out] response Planned path in the `map` frame. Empty if planning failed, since the
   *   response has no success flag.
   */
  void handle_plan_path(const std::shared_ptr<PlanPath::Request> request, std::shared_ptr<PlanPath::Response> response)
  {
    RCLCPP_INFO(
        this->get_logger(), "Received plan request: start (%.2f, %.2f, %.2f) -> goal (%.2f, %.2f, %.2f)",
        request->start_pose.position.x, request->start_pose.position.y, request->start_pose.position.z,
        request->goal_pose.position.x, request->goal_pose.position.y, request->goal_pose.position.z
    );

    nav_msgs::msg::Path& path = response->path;
    
    path.header.stamp = this->now();

  nav_msgs::msg::OccupancyGrid::ConstSharedPtr costmap;

  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    costmap = latest_costmap_;
  }

  if (!costmap || costmap->header.frame_id.empty()) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan: no valid costmap received");
    path.poses.clear();
    return;
  }

  path.header.frame_id = costmap->header.frame_id;

  auto* astar = static_cast<AStarPlanner*>(planner_.get());
  astar->set_costmap(costmap);

    if (planner_->plan(request->start_pose, request->goal_pose, path)) {
      RCLCPP_INFO(this->get_logger(), "Planned path with %zu poses", path.poses.size());
      /// Publishing path if the response succeeded
      path_pub_->publish(path);
    } else {
      /// The response has no success flag, so a failed plan is reported as an empty path
      path.poses.clear();
      path_pub_->publish(path);
      RCLCPP_WARN(this->get_logger(), "Planning failed, returning an empty path");
    }
  }

};  // class PlannerService

}  // namespace pennair_planner

RCLCPP_COMPONENTS_REGISTER_NODE(pennair_planner::PlannerService)
