#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

/*
Abstract class interface for all payload controllers to implement
All main functionalities should be virtual functions here
*/

class Controller {
    public:
        virtual ~Controller() = default;
        virtual void initialize(std::shared_ptr<rclcpp::Node> node) = 0;
        virtual void drive_command(double linear, double angular) = 0;
        // TODO: add servo command functionality
        // TODO: add position_setpoint functionality
};

#endif