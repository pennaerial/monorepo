#include "payload/gpio_controller.hpp"

GPIOController::GPIOController() {
    //setup gpio pins
}

void GPIOController::initialize(std::shared_ptr<rclcpp::Node> node) {

}

void GPIOController::drive_command(double linear, double angular) {

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(GPIOController, Controller)
