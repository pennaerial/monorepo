#include <functional>
#include <memory>

#include "nav_msgs/msg/path.hpp"
#include "pennair_planner/path_planner.hpp"
#include "pennair_planner/srv/plan_path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

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
        planner_(std::make_unique<StraightLinePlanner>())  // swap in a different PathPlanner here
  {
    this->service_ = this->create_service<PlanPath>(
        "plan_path", std::bind(&PlannerService::handle_plan_path, this, std::placeholders::_1, std::placeholders::_2)
    );
  }

private:
  rclcpp::Service<PlanPath>::SharedPtr service_;
  std::unique_ptr<PathPlanner> planner_;

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
    path.header.frame_id = "map";

    if (planner_->plan(request->start_pose, request->goal_pose, path)) {
      RCLCPP_INFO(this->get_logger(), "Planned path with %zu poses", path.poses.size());
    } else {
      // The response has no success flag, so a failed plan is reported as an empty path
      path.poses.clear();
      RCLCPP_WARN(this->get_logger(), "Planning failed, returning an empty path");
    }
  }

};  // class PlannerService

}  // namespace pennair_planner

RCLCPP_COMPONENTS_REGISTER_NODE(pennair_planner::PlannerService)
