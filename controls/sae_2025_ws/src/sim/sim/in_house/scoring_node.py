#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition
from std_msgs.msg import Float32, String
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
        
        # Scoring parameters
        self.hoop_radius = self.get_parameter('hoop_tolerance').get_parameter_value().double_value
        self.min_altitude = 0.5  # minimum altitude to score
        self.max_altitude = 10.0  # maximum altitude to score
        
        # Course data
        self.hoop_poses: List[Tuple[float, float, float]] = []
        self.passed_hoops: List[bool] = []
        self.current_score = 0
        self.start_time = time.time()
        
        # UAV tracking
        self.uav_position: Optional[Tuple[float, float, float]] = None
        self.position_history: List[Tuple[float, float, float, float]] = []  # x, y, z, timestamp
        self.max_history = 100  # Keep last 100 positions
        
        # Publishers
        self.score_publisher = self.create_publisher(Float32, '/scoring/results', 10)
        self.status_publisher = self.create_publisher(String, '/scoring/status', 10)
        
        # Subscriber for UAV position
        self.position_subscription = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.position_callback,
            10
        )
        
        # Timer for periodic scoring updates
        self.create_timer(0.1, self.update_scoring)  # 10Hz
        
        # Load hoop positions from parameters
        self.load_hoop_positions_from_params()
        
        self.get_logger().info("Scoring node initialized")
    
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
            
            # Convert to list of tuples
            hoop_positions = []
            for i in range(0, len(hoop_positions_list), 3):
                if i + 2 < len(hoop_positions_list):
                    hoop_positions.append((
                        hoop_positions_list[i],
                        hoop_positions_list[i + 1],
                        hoop_positions_list[i + 2]
                    ))
            
            self.set_course_hoops(hoop_positions)
            self.get_logger().info(f"Loaded {len(hoop_positions)} hoops from parameters")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load hoop positions from parameters: {e}")
            raise
    
    def set_course_hoops(self, hoop_poses: List[Tuple[float, float, float]]):
        """
        Set the hoop positions for scoring.
        
        Args:
            hoop_poses: List of (x, y, z) positions for each hoop
        """
        self.hoop_poses = hoop_poses
        self.passed_hoops = [False] * len(hoop_poses)
        self.get_logger().info(f"Course set with {len(hoop_poses)} hoops")
    
    def position_callback(self, msg: VehicleLocalPosition):
        """Callback for UAV position updates."""
        self.uav_position = (msg.x, msg.y, msg.z)
        
        # Add to position history
        current_time = time.time()
        self.position_history.append((msg.x, msg.y, msg.z, current_time))
        
        # Keep only recent history
        if len(self.position_history) > self.max_history:
            self.position_history.pop(0)
    
    def update_scoring(self):
        """Periodic scoring update."""
        if not self.uav_position or not self.hoop_poses:
            return
        
        x, y, z = self.uav_position
        
        # Check each hoop for passage
        for i, hoop_pos in enumerate(self.hoop_poses):
            if not self.passed_hoops[i]:
                hoop_x, hoop_y, hoop_z = hoop_pos
                
                # Calculate distance to hoop center
                distance = math.sqrt((x - hoop_x)**2 + (y - hoop_y)**2 + (z - hoop_z)**2)
                
                # Check if UAV passed through hoop
                if (distance <= self.hoop_radius and 
                    self.min_altitude <= z <= self.max_altitude):
                    
                    self.passed_hoops[i] = True
                    self.current_score += 1
                    
                    self.get_logger().info(f"Hoop {i+1} passed! Score: {self.current_score}")
                    
                    # Publish score update
                    self.publish_score()
                    self.publish_status(f"Hoop {i+1} passed")
        
        # Publish periodic score updates
        self.publish_score()
    
    def publish_score(self):
        """Publish current score."""
        score_msg = Float32()
        score_msg.data = float(self.current_score)
        self.score_publisher.publish(score_msg)
    
    def publish_status(self, status: str):
        """Publish status message."""
        status_msg = String()
        status_msg.data = status
        self.status_publisher.publish(status_msg)
    
    def get_score(self) -> int:
        """Get current score."""
        return self.current_score
    
    def get_completion_percentage(self) -> float:
        """Get course completion percentage."""
        if not self.hoop_poses:
            return 0.0
        return (self.current_score / len(self.hoop_poses)) * 100.0
    
    def reset_scoring(self):
        """Reset scoring for new run."""
        self.current_score = 0
        self.passed_hoops = [False] * len(self.hoop_poses)
        self.start_time = time.time()
        self.position_history.clear()
        self.get_logger().info("Scoring reset")


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