#ifndef GPIO_CONTROLLER_HPP
#define GPIO_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include "payload/controller.hpp"

class GPIOController : public Controller {
    public:
        GPIOController(rclcpp::Logger logger);
        void drive_command(double linear, double angular) override;
        void initialize(std::shared_ptr<rclcpp::Node> node) override;
    
    private:
        rclcpp::Logger logger_;
        std::shared_ptr<rclcpp::Node> node_;



};

#endif 