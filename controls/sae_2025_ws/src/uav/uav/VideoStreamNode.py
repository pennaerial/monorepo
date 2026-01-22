#!/usr/bin/env python3
"""
VideoStreamNode: Streams camera feed from ROS 2 to QGroundControl via GStreamer
Works for both simulation and hardware (subscribes to /camera topic)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import subprocess
import sys


class VideoStreamNode(Node):
    """
    ROS 2 node that subscribes to camera images and streams them to QGroundControl
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
        
        # Initialize GStreamer pipeline
        self.gst_pipeline = None
        self.setup_gstreamer()
        
        # Subscribe to camera topic
        self.image_subscription = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10
        )
        
        self.get_logger().info('Video stream node initialized')
    
    def setup_gstreamer(self):
        """
        Set up GStreamer pipeline for H.264 UDP streaming.
        """
        try:
            # GStreamer pipeline for H.264 encoding and UDP streaming
            # This pipeline:
            # 1. Receives raw video from appsrc
            # 2. Converts to proper format
            # 3. Encodes with H.264 (x264enc with ultrafast preset for low latency)
            # 4. Packages as RTP
            # 5. Sends via UDP to QGroundControl
            
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
            
            # Import GStreamer Python bindings
            import gi
            gi.require_version('Gst', '1.0')
            from gi.repository import Gst
            
            Gst.init(None)
            self.gst_pipeline = Gst.parse_launch(gst_command)
            self.gst_pipeline.set_state(Gst.State.PLAYING)
            
            self.appsrc = self.gst_pipeline.get_by_name('source')
            
            self.get_logger().info('GStreamer pipeline initialized successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize GStreamer: {e}')
            self.get_logger().error('Make sure gstreamer and python3-gst-1.0 are installed')
            self.gst_pipeline = None
    
    def image_callback(self, msg: Image):
        """
        Callback for receiving camera images and pushing to GStreamer pipeline.
        """
        if self.gst_pipeline is None:
            return
        
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Resize if necessary
            if cv_image.shape[1] != self.width or cv_image.shape[0] != self.height:
                cv_image = cv2.resize(cv_image, (self.width, self.height))
            
            # Push frame to GStreamer
            import gi
            gi.require_version('Gst', '1.0')
            from gi.repository import Gst, GLib
            
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
                self.get_logger().warn(f'Failed to push buffer: {retval}')
                
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
