#include <rclcpp/rclcpp.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>
#include "payload/payload.hpp"
#include <string>

class SimController : public Controller {
    public:
        SimController(const std::string& gz_drive_topic) {
            gz_node_ = std::make_shared<gz::transport::Node>();
            gz_drive_publisher_ = gz_node_->Advertise<gz::msgs::Twist>(gz_drive_topic);
        }

        void drive_command(double linear, double angular) override {
            gz::msgs::Twist msg;
            msg.mutable_linear()->set_x(linear); //head-on direction
            msg.mutable_angular()->set_z(angular); //positive for left, negative for right
            gz_drive_publisher_.Publish(msg);
        }
    
    private:
        std::shared_ptr<gz::transport::Node> gz_node_;
        gz::transport::Node::Publisher gz_drive_publisher_;

};