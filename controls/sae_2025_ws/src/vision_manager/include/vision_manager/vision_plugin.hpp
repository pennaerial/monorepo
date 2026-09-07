#pragma once

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/image.hpp"

namespace pennair_vision
{

class VisionPlugin
{
public:
  virtual ~VisionPlugin() = default;

  /// Initialize the plugin by passing in ROS node
  virtual void initialize(rclcpp::Node::SharedPtr node) = 0;

  /// Run the VisionPlugin impl and publish the results
  virtual void process(sensor_msgs::msg::Image::ConstSharedPtr image) = 0;


private:
  /// node ptr for params and creating publishers
  rclcpp::Node::SharedPtr node_;
};


}  // namespace pennair_vision
