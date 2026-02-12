#!/usr/bin/env python3
"""
Relays payload camera image and camera_info to fixed topic names so tools
like rqt_image_view can show the feed. Subscribe to /payload_camera to view.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo


def main(args=None):
    rclpy.init(args=args)
    node = Node("payload_camera_relay")

    payload_name = node.declare_parameter("payload_name", "payload_0").value
    in_image = f"/{payload_name}/camera"
    in_info = f"/{payload_name}/camera_info"
    out_image = "/payload_camera"
    out_info = "/payload_camera_info"

    pub_image = node.create_publisher(Image, out_image, 10)
    pub_info = node.create_publisher(CameraInfo, out_info, 10)

    def cb_image(msg: Image) -> None:
        pub_image.publish(msg)

    def cb_info(msg: CameraInfo) -> None:
        pub_info.publish(msg)

    node.create_subscription(Image, in_image, cb_image, 10)
    node.create_subscription(CameraInfo, in_info, cb_info, 10)

    node.get_logger().info(
        f"Relaying {in_image} -> {out_image}, {in_info} -> {out_info}"
    )
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
