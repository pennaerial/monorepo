#include <rclcpp/rclcpp.hpp>
#include "custom_sim_bridge/convert.hpp"
#include "custom_sim_bridge/custom_sim_bridge_parameters.hpp"
#include "gz/transport/Node.hh"

namespace custom_sim_bridge {

using AttachDetach = sim_interfaces::srv::AttachDetach;

class AttachDetachBridge 
: public rclcpp::Node {
    public:
        AttachDetachBridge() : rclcpp::Node("attach_detach_bridge") { 
            param_listener_ = std::make_shared<custom_sim_bridge::ParamListener>(this);
            params_ = param_listener_->get_params();
            //validate necessary params
            if (params_.ros_service_name.empty() || params_.gz_service_name.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Both ros_service_name and gz_service_name must be non-empty");
                throw std::invalid_argument("Invalid bridge parameters");
            }

            RCLCPP_INFO(this->get_logger(), "ROS AttachDetach srv at: %s", params_.ros_service_name.c_str());
            RCLCPP_INFO(this->get_logger(), "GZ AttachDetach srv at: %s", params_.gz_service_name.c_str());

            gz_node_ = std::make_shared<gz::transport::Node>();
            attach_detach_ros_service_ = this->create_service<AttachDetach>(params_.ros_service_name, std::bind(
                &AttachDetachBridge::attach_detach_callback, this, std::placeholders::_1, std::placeholders::_2)
            );
        }

        void attach_detach_callback(AttachDetach::Request::SharedPtr request, AttachDetach::Response::SharedPtr response) {
            auto gz_req = convert_to_gz(request);
            gz::custom_msgs::AttachDetachResponse gz_resp;
            bool result;
            bool executed = gz_node_->Request(params_.gz_service_name, gz_req, 5000, gz_resp, result);

            if (executed && result) {
                response->success = gz_resp.success();
                response->message = gz_resp.message();
            } else {
                response->success = false;
                response->message = executed ? "Gz service returned failure" : "Gz service call timed out";
            }
        }


    private:
        custom_sim_bridge::Params params_;
        std::shared_ptr<custom_sim_bridge::ParamListener> param_listener_;
        std::shared_ptr<gz::transport::Node> gz_node_;
        rclcpp::Service<AttachDetach>::SharedPtr attach_detach_ros_service_;

};


}


#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(custom_sim_bridge::AttachDetachBridge, rclcpp::Node)
