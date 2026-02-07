#!/usr/bin/env python3
"""
VideoStreamNode: Streams camera feed from ROS 2 to QGroundControl via GStreamer
Works for both simulation and hardware (subscribes to /camera topic)
Draws bounding boxes around detected AprilTags with IDs and distance
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from uav_interfaces.msg import AprilTagDetectionArray
from cv_bridge import CvBridge
import cv2
import numpy as np


class VideoStreamNode(Node):
    """
    ROS 2 node that subscribes to camera images and AprilTag detections,
    draws bounding boxes on the video, and streams to QGroundControl
    using GStreamer H.264 UDP pipeline.
    """

    def __init__(self):
        super().__init__('video_stream_node')

        # Declare parameters
        self.declare_parameter('qgc_ip', '127.0.0.1')
        self.declare_parameter('qgc_port', 5600)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('framerate', 30)
        self.declare_parameter('bitrate', 800)  # kbps

        # Get parameters
        self.qgc_ip = self.get_parameter('qgc_ip').value
        self.qgc_port = self.get_parameter('qgc_port').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.framerate = self.get_parameter('framerate').value
        self.bitrate = self.get_parameter('bitrate').value

        self.get_logger().info(f'Starting video stream to {self.qgc_ip}:{self.qgc_port}')

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Store latest AprilTag detections
        self.latest_detections = None

        # Initialize GStreamer pipeline
        self.gst_pipeline = None
        self.appsrc = None
        self.setup_gstreamer()

        # Subscribe to camera topic
        self.image_subscription = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10
        )

        # Subscribe to AprilTag detections
        self.apriltag_subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag_detections',
            self.apriltag_callback,
            10
        )

        self.get_logger().info('Video stream node initialized')

    def setup_gstreamer(self):
        """
        Set up GStreamer pipeline for H.264 UDP streaming.
        """
        try:
            # Import GStreamer Python bindings
            import gi
            gi.require_version('Gst', '1.0')
            from gi.repository import Gst

            Gst.init(None)

            # GStreamer pipeline for H.264 encoding and UDP streaming
            gst_command = (
                f'appsrc name=source is-live=true block=true format=GST_FORMAT_TIME '
                f'caps=video/x-raw,format=BGR,width={self.width},height={self.height},'
                f'framerate={self.framerate}/1 ! '
                f'videoconvert ! '
                f'video/x-raw,format=I420 ! '
                f'x264enc tune=zerolatency bitrate={self.bitrate} speed-preset=ultrafast key-int-max=30 ! '
                f'rtph264pay config-interval=1 pt=96 ! '
                f'udpsink host={self.qgc_ip} port={self.qgc_port} sync=false'
            )

            self.gst_pipeline = Gst.parse_launch(gst_command)
            self.gst_pipeline.set_state(Gst.State.PLAYING)

            self.appsrc = self.gst_pipeline.get_by_name('source')

            self.get_logger().info('GStreamer pipeline initialized successfully')

        except Exception as e:
            self.get_logger().error(f'Failed to initialize GStreamer: {e}')
            self.get_logger().error('Make sure gstreamer and python3-gst-1.0 are installed')
            self.gst_pipeline = None

    def apriltag_callback(self, msg: AprilTagDetectionArray):
        """
        Callback for receiving AprilTag detections.
        """
        self.latest_detections = msg

    def draw_apriltag_boxes(self, image: np.ndarray) -> np.ndarray:
        """
        Draw bounding boxes around detected AprilTags.

        Args:
            image: Input BGR image

        Returns:
            Image with bounding boxes drawn
        """
        if self.latest_detections is None or len(self.latest_detections.detections) == 0:
            return image

        overlay = image.copy()

        for detection in self.latest_detections.detections:
            tag_id = detection.tag_id
            x = int(detection.x)
            y = int(detection.y)
            distance = detection.distance

            # Estimate box size based on distance (closer = bigger box)
            # Assuming tag size of ~165mm, this gives rough pixel size
            box_size = int(100 / max(distance, 0.5))  # Prevent division by zero
            box_size = min(max(box_size, 20), 200)  # Clamp between 20-200 pixels

            # Calculate bounding box corners
            x1 = x - box_size // 2
            y1 = y - box_size // 2
            x2 = x + box_size // 2
            y2 = y + box_size // 2

            # Draw filled semi-transparent rectangle
            cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 0), -1)

            # Blend with original image for transparency
            alpha = 0.3
            cv2.addWeighted(overlay, alpha, image, 1 - alpha, 0, image)

            # Draw border
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 3)

            # Draw center point
            cv2.circle(image, (x, y), 5, (0, 0, 255), -1)

            # Prepare text labels
            id_text = f"ID: {tag_id}"
            dist_text = f"{distance:.2f}m"

            # Calculate text sizes for background rectangles
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.6
            thickness = 2

            (id_w, id_h), _ = cv2.getTextSize(id_text, font, font_scale, thickness)
            (dist_w, dist_h), _ = cv2.getTextSize(dist_text, font, font_scale, thickness)

            # Draw background rectangles for text
            padding = 5
            cv2.rectangle(image,
                         (x1, y1 - id_h - 2 * padding),
                         (x1 + id_w + 2 * padding, y1),
                         (0, 255, 0), -1)
            cv2.rectangle(image,
                         (x1, y1 - id_h - dist_h - 3 * padding),
                         (x1 + dist_w + 2 * padding, y1 - id_h - 2 * padding),
                         (0, 255, 0), -1)

            # Draw text labels
            cv2.putText(image, id_text,
                       (x1 + padding, y1 - padding),
                       font, font_scale, (0, 0, 0), thickness)
            cv2.putText(image, dist_text,
                       (x1 + padding, y1 - id_h - 2 * padding),
                       font, font_scale, (0, 0, 0), thickness)

        # Add detection count overlay in top-left corner
        if len(self.latest_detections.detections) > 0:
            count_text = f"AprilTags: {len(self.latest_detections.detections)}"
            (count_w, count_h), _ = cv2.getTextSize(count_text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
            cv2.rectangle(image, (5, 5), (count_w + 15, count_h + 15), (0, 255, 0), -1)
            cv2.putText(image, count_text, (10, count_h + 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)

        return image

    def image_callback(self, msg: Image):
        """
        Callback for receiving camera images, drawing AprilTag boxes,
        and pushing to GStreamer pipeline.
        """
        if self.gst_pipeline is None:
            return

        try:
            # Convert ROS Image message to OpenCV format (handle both sim and hardware)
            if hasattr(msg, 'data'):
                # Check if we're in simulation mode (direct numpy conversion)
                if isinstance(msg.data, bytes):
                    img_data = np.frombuffer(msg.data, dtype=np.uint8)
                    cv_image = img_data.reshape((msg.height, msg.width, 3))
                else:
                    cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Resize if necessary
            if cv_image.shape[1] != self.width or cv_image.shape[0] != self.height:
                cv_image = cv2.resize(cv_image, (self.width, self.height))

            # Draw AprilTag bounding boxes
            cv_image = self.draw_apriltag_boxes(cv_image)

            # Push frame to GStreamer
            import gi
            gi.require_version('Gst', '1.0')
            from gi.repository import Gst

            data = cv_image.tobytes()
            buf = Gst.Buffer.new_allocate(None, len(data), None)
            buf.fill(0, data)
            buf.duration = int(1000000000 / self.framerate)  # nanoseconds

            # Set timestamp
            timestamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
            buf.pts = timestamp
            buf.dts = timestamp

            # Push buffer to appsrc
            retval = self.appsrc.emit('push-buffer', buf)

            if retval != Gst.FlowReturn.OK:
                self.get_logger().warn(f'Failed to push buffer: {retval}', throttle_duration_sec=5.0)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def destroy_node(self):
        """Clean up GStreamer pipeline on shutdown."""
        if self.gst_pipeline is not None:
            import gi
            gi.require_version('Gst', '1.0')
            from gi.repository import Gst

            self.gst_pipeline.set_state(Gst.State.NULL)
            self.get_logger().info('GStreamer pipeline stopped')

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = VideoStreamNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in video stream node: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
