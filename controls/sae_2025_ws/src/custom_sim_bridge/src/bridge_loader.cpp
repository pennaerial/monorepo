#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Usage: bridge_loader <plugin_class_name>" << std::endl;
        std::cerr << "Example: bridge_loader AttachDetachBridge" << std::endl;
        return 1;
    }

    std::string bridge_type = std::string("custom_sim_bridge::") + argv[1];
    rclcpp::init(argc, argv);

    try {
        pluginlib::ClassLoader<rclcpp::Node> loader("rclcpp", "rclcpp::Node");
        auto node = loader.createSharedInstance(bridge_type);
        RCLCPP_INFO(node->get_logger(), "Loaded bridge plugin: %s", bridge_type.c_str());
        rclcpp::spin(node);
    } catch (const pluginlib::PluginlibException& e) {
        std::cerr << "CUSTOM SIM BRIDGE | Failed to load bridge plugin '" << bridge_type << "': " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
