#include "payload/gpio_controller.hpp"

GPIOController::GPIOController(rclcpp::Logger logger)
: logger_(logger) {
    //setup gpio pins
}

void GPIOController::drive_command(double linear, double angular) {
    
}
