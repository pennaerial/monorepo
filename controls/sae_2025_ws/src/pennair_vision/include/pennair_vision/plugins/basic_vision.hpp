#pragma once

#include "pennair_vision/vision_plugin.hpp"
#include <rclcpp/rclcpp.hpp>

namespace pennair_vision
{

class BasicVision : public VisionPlugin
{
public:
  /// See VisionPlugin
  void initialize(const rclcpp::Node::SharedPtr& node) override;

  /// See VisionPlugin
  void process(sensor_msgs::msg::Image::ConstSharedPtr image) override;
};

}; // pennair_vision
