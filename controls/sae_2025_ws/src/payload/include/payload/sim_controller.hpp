#ifndef SIM_CONTROLLER_HPP
#define SIM_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>
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
};

#endif
