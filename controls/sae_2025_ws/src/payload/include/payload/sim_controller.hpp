#ifndef SIM_CONTROLLER_HPP
#define SIM_CONTROLLER_HPP


#include <rclcpp/rclcpp.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>
#include "payload/payload.hpp"
#include <string>

class SimController : public Controller {
    public:
        SimController(const std::string& gz_drive_topic, rclcpp::Logger logger);
        void drive_command(double linear, double angular) override;
    
    private:
        std::shared_ptr<gz::transport::Node> gz_node_;
        gz::transport::Node::Publisher gz_drive_publisher_;
        rclcpp::Logger logger_;

};

#endif