#include "rclcpp/rclcpp.hpp"
#include "payload/payload_controller.hpp"


int main(int argc, char **argv){
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PayloadController>("payload_0");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}