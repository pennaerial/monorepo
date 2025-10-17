#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Float32, String, Float32MultiArray
from geometry_msgs.msg import Point
from typing import List, Tuple, Optional, Dict, Any
import math
import time


class ScoringNode(Node):
    """
    ROS2 node for in-house competition scoring.
    Tracks UAV position and scores hoop passages.
    """
    
    def __init__(self):
        super().__init__('scoring_node')
        
        # Declare parameters
        self.declare_parameter('hoop_positions', '[]')
        self.declare_parameter('hoop_tolerance', 1.5)
        self.declare_parameter('competition_type', 'in_house')
        self.declare_parameter('competition_name', 'test')
        self.declare_parameter('course_type', 'straight')
        
        # Course data
        self.hoop_poses: List[Tuple[float, float, float, float, float, float]] = []
        self.passed_hoops: List[bool] = []
        self.current_score = 0
        self.start_time = time.time()
        
        # UAV tracking
        self.uav_position: Optional[Tuple[float, float, float]] = None
        self.position_history: List[Tuple[float, float, float, float]] = []  # x, y, z, timestamp
        self.max_history = 100  # Keep last 100 positions
        self.prev_position: Optional[Tuple[float, float, float]] = None
        
        # Publishers
        self.score_publisher = self.create_publisher(Float32, '/scoring/results', 10)
        self.status_publisher = self.create_publisher(String, '/scoring/status', 10)
        self.drone_publisher = self.create_publisher(Float32MultiArray, '/scoring/drone_position', 10)
        
        # Subscriber for UAV position (match PX4 best-effort sensor QoS)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.position_subscription = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.position_callback,
            qos
        )
        
        # Timer for periodic scoring updates
        self.create_timer(0.1, self.update_scoring)  # 10Hz
        
        self.load_hoop_positions_from_params()
        self.get_logger().info("Scoring node initialized.")
    
    def load_hoop_positions_from_params(self):
        """Load hoop positions from ROS2 parameters."""
        try:
            hoop_positions_str = self.get_parameter('hoop_positions').get_parameter_value().string_value
            
            if hoop_positions_str == '[]' or not hoop_positions_str:
                self.get_logger().error("No hoop positions provided in parameters!")
                raise ValueError("No hoop positions provided in parameters")
            
            # Parse string representation of list
            import ast
            hoop_positions_list = ast.literal_eval(hoop_positions_str)
            
            # Convert to list of tuples (now expecting 6 values per hoop)
            hoop_positions = []
            for i in range(0, len(hoop_positions_list), 6): 
                if i + 5 < len(hoop_positions_list):  
                    hoop_positions.append((
                        hoop_positions_list[i],     # x
                        hoop_positions_list[i + 1], # y
                        hoop_positions_list[i + 2], # z
                        hoop_positions_list[i + 3], # roll
                        hoop_positions_list[i + 4], # pitch
                        hoop_positions_list[i + 5]  # yaw
                    ))
            
            self.set_course_hoops(hoop_positions)
            self.get_logger().info(f"Loaded {len(hoop_positions)} hoops from parameters")
            
            # Log all hoop poses with full orientation
            for i, hoop in enumerate(hoop_positions):
                x, y, z, roll, pitch, yaw = hoop
                self.get_logger().info(f"Hoop {i+1}: x={x:.2f}, y={y:.2f}, z={z:.2f}, roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load hoop positions from parameters: {e}")
            raise
        
    def set_course_hoops(self, hoop_poses: List[Tuple[float, float, float, float, float, float]]):
        """Set the hoop positions for scoring."""
        self.hoop_poses = hoop_poses
        self.passed_hoops = [False] * len(hoop_poses)
        self.get_logger().info(f"Course set with {len(hoop_poses)} hoops")

    def update_scoring(self):
        """Check if drone passes through any hoop and publish status."""
        if not self.uav_position or not self.hoop_poses:
            return
        
        x, y, z = self.uav_position
        
        # Check each hoop for passage
        for i, hoop in enumerate(self.hoop_poses):
            if self.passed_hoops[i]:  # Skip if already passed
                continue
                
            hoop_x, hoop_y, hoop_z, roll, pitch, yaw = hoop
            
            # Calculate distance to hoop center
            distance = math.sqrt((x - hoop_x)**2 + (y - hoop_y)**2 + (z - hoop_z)**2)
            
            # Get tolerance from parameters (with generous buffer)
            tolerance = self.get_parameter('hoop_tolerance').get_parameter_value().double_value
            
            # Check if drone is within hoop (with generous buffer)
            if distance <= tolerance:
                # Mark hoop as passed
                self.passed_hoops[i] = True
                self.current_score += 1
                
                # Publish status message
                status_msg = String()
                status_msg.data = f"Hoop {i+1} passed! Score: {self.current_score}"
                self.status_publisher.publish(status_msg)
                
                # Publish score update
                score_msg = Float32()
                score_msg.data = float(self.current_score)
                self.score_publisher.publish(score_msg)
                
                # Log the achievement
                self.get_logger().info(f"🎯 HOOP {i+1} PASSED! Distance: {distance:.2f}m, Score: {self.current_score}")
                
                # Log drone position when passing
                self.get_logger().info(f"Drone at: x={x:.2f}, y={y:.2f}, z={z:.2f}")
                self.get_logger().info(f"Hoop {i+1} at: x={hoop_x:.2f}, y={hoop_y:.2f}, z={hoop_z:.2f}")
        
        # Publish periodic status (even if no hoop passed)
        if self.uav_position:
            # Throttled position logging
            self.get_logger().info(f"UAV Position: x={x:.2f}, y={y:.2f}, z={z:.2f}", throttle_duration_sec=5.0)
    
    
    def position_callback(self, msg: VehicleLocalPosition):
        """Callback for UAV position updates.
        PX4 publishes local position in NED (z positive down). Convert to Up for scoring.
        """
        # Convert to Up axis for z
        z_up = -float(msg.z)
        x = float(msg.x)
        y = float(msg.y)
        self.prev_position = self.uav_position
        self.uav_position = (x, y, z_up)
        
        # Add to position history
        current_time = time.time()
        self.position_history.append((x, y, z_up, current_time))
        
        # Keep only recent history
        if len(self.position_history) > self.max_history:
            self.position_history.pop(0)
        
        # Publish drone position
        drone_position = Float32MultiArray()
        drone_position.data = [x, y, z_up]
        self.drone_publisher.publish(drone_position)
    
   
def main(args=None):
    """Main function for scoring node."""
    rclpy.init(args=args)
    
    scoring_node = ScoringNode()
    
    try:
        rclpy.spin(scoring_node)
    except KeyboardInterrupt:
        pass
    finally:
        scoring_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()