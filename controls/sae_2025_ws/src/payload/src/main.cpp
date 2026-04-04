#include <cstdlib>
#include <csignal>

#include "payload/payload.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {
void payload_signal_exit(int) {
    std::_Exit(0);
}
}  // namespace

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    std::signal(SIGINT, payload_signal_exit);
    std::signal(SIGTERM, payload_signal_exit);
    bool interrupted_shutdown = false;

    {
        auto node = std::make_shared<Payload>("payload_main");
        node->init();
        rclcpp::spin(node);
        interrupted_shutdown = !rclcpp::ok();
        node.reset();
    }
    rclcpp::shutdown();
    if (interrupted_shutdown) {
        std::_Exit(0);
    }

    return 0;
}
