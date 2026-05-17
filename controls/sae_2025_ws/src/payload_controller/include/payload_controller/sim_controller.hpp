#ifndef SIM_CONTROLLER_HPP
#define SIM_CONTROLLER_HPP

#include <atomic>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>
#include "payload_controller/controller.hpp"
#include "payload_controller/payload_controller_parameters.hpp"

class SimController : public Controller
{
public:
  SimController();
  void initialize(rclcpp::Node * node) override;
  void drive_command(double linear, double angular) override;
  void servo_command(double degree) override;
  void safe_shutdown() override;

  std::shared_ptr<payload::ParamListener> payload_params_listener_;
  payload::Params payload_params_;

private:
  void publish_drive_command(double linear, double angular);

  std::atomic<bool> shutdown_requested_ {false};
  std::shared_ptr<gz::transport::Node> gz_node_;
  gz::transport::Node::Publisher gz_drive_publisher_;
};

#endif
