#include "rclcpp/rclcpp.hpp"
#include "payload/payload.hpp"


int main(int argc, char **argv){
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Payload>("payload_main");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}