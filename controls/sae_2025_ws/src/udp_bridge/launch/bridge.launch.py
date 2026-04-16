from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "my_port",
                description="UDP port this node listens on (its identity, e.g. 5000)",
            ),
            DeclareLaunchArgument(
                "namespace",
                description="ROS namespace for this bridge node (e.g. 'pi0', 'laptop')",
            ),
            DeclareLaunchArgument(
                "all_ports",
                default_value="[5000, 5001, 5002, 5003]",
                description="All node ports in the fleet (including my_port)",
            ),
            DeclareLaunchArgument(
                "broadcast_ip",
                default_value="10.42.0.255",
                description="Subnet broadcast address",
            ),
            DeclareLaunchArgument(
                "topics",
                default_value="[]",
                description=(
                    "Topics to bridge, as 'topic_name:pkg/msg/MsgType' entries. "
                    "Example: \"['test:uav_interfaces/msg/UdpTest']\""
                ),
            ),
            Node(
                package="udp_bridge",
                executable="bridge_node",
                name="udp_bridge",
                namespace=LaunchConfiguration("namespace"),
                parameters=[
                    {
                        "my_port": LaunchConfiguration("my_port"),
                        "all_ports": LaunchConfiguration("all_ports"),
                        "broadcast_ip": LaunchConfiguration("broadcast_ip"),
                        "topics": LaunchConfiguration("topics"),
                    }
                ],
                output="screen",
            ),
        ]
    )
