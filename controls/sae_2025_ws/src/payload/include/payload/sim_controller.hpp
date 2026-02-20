#ifndef SIM_CONTROLLER_HPP
#define SIM_CONTROLLER_HPP


#include <rclcpp/rclcpp.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/camera_info.pb.h>
#include "ros_gz_bridge/convert.hpp"
#include "payload/controller.hpp"
#include "payload/payload_parameters.hpp"
#include <string>

class SimController : public Controller {
    public:
        SimController();
        void initialize(std::shared_ptr<rclcpp::Node> node) override;
        void drive_command(double linear, double angular) override;
    
    private:
        std::shared_ptr<rclcpp::Node> node_;

        std::shared_ptr<payload::ParamListener> payload_params_listener_;
        payload::Params payload_params_;

        std::shared_ptr<gz::transport::Node> gz_node_;
        gz::transport::Node::Publisher gz_drive_publisher_;


        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ros_camera_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr ros_camera_info_publisher_;

        void gz_camera_callback(const gz::msgs::Image& gz_msg);
        void gz_camera_info_callback(const gz::msgs::CameraInfo& gz_msg);

};

#endif