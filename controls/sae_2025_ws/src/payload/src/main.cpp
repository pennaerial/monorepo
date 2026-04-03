#include "rclcpp/rclcpp.hpp"
#include "payload/payload.hpp"


int main(int argc, char **argv){
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("payload");
    auto payload = std::make_shared<Payload>(node);
    payload->init();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
