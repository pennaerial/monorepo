#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Float32, String, Float32MultiArray, ColorRGBA, Header
from geometry_msgs.msg import Point
from ros_gz_interfaces.msg import MaterialColor, Entity
from typing import List, Tuple, Optional, Dict, Any
import math
import time


class ScoringNode(Node):
    """
    ROS2 node for in-house competition scoring.
    Tracks UAV position and scores hoop passages using directional detection.
    """
    
    def __init__(self):
        super().__init__('scoring_node')
        
        # Declare parameters
        self.declare_parameter('hoop_positions', '[]')
        self.declare_parameter('hoop_tolerance', 1.5) 
        self.declare_parameter('competition_type', 'in_house')
        self.declare_parameter('competition_name', 'test')
        self.declare_parameter('course_type', 'straight')
        self.declare_parameter('world_name', 'custom')  # Gazebo world name
        
        # landing bonus parameters
        self.declare_parameter('landing_bonus', 5.0)  
        self.declare_parameter('landing_tolerance', 2.0) 
        self.declare_parameter('landing_altitude_max', 0.4)  

        
        # Course data
        self.hoop_poses: List[Tuple[float, float, float, float, float, float]] = []
        self.passed_hoops: List[bool] = []
        self.current_score = 0.0
        self.start_time = time.time()
        
        # UAV tracking
        self.uav_position: Optional[Tuple[float, float, float]] = None
        self.position_history: List[Tuple[float, float, float, float]] = []
        self.max_history = 100
        self.prev_position: Optional[Tuple[float, float, float]] = None
        
        # Directional detection tracking
        self.drone_last_side: List[int] = []  # Track which side of each hoop drone is on
        self.landed_at_origin = False
        self.landing_bonus_awarded = False
        
        # Publishers
        self.score_publisher = self.create_publisher(Float32, '/scoring/results', 10)
        self.status_publisher = self.create_publisher(String, '/scoring/status', 10)
        self.drone_publisher = self.create_publisher(Float32MultiArray, '/scoring/drone_position', 10)
        
        # Publisher for hoop color updates (MaterialColor messages to Gazebo)
        world_name = self.get_parameter('world_name').get_parameter_value().string_value
        self.visual_config_topic = f'/world/{world_name}/visual_config'
        self.visual_config_publisher = self.create_publisher(
            MaterialColor, 
            self.visual_config_topic, 
            10
        )
        self.get_logger().info(f"Initialized visual config publisher on topic: {self.visual_config_topic}")
        
        # Track last hoop color update to throttle updates
        self.last_color_update_time = 0.0
        self.color_update_interval = 0.5  # Update colors at 2Hz max
        self.initial_color_setup_done = False  # Track if initial setup is complete
        
        # Subscriber
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
        
        # Timer for score updating
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
            
            import ast
            hoop_positions_list = ast.literal_eval(hoop_positions_str)
            
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
            
            for i, hoop in enumerate(hoop_positions):
                x, y, z, roll, pitch, yaw = hoop
                self.get_logger().info(f"Hoop {i+1}: x={x:.2f}, y={y:.2f}, z={z:.2f}, roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load hoop positions from parameters: {e}")
            raise
        
    def set_course_hoops(self, hoop_poses: List[Tuple[float, float, float, float, float, float]]):
        """Set the hoop positions for scoring. (UPDATED)"""
        self.hoop_poses = hoop_poses
        self.passed_hoops = [False] * len(hoop_poses)
        # NEW: Initialize side tracking for each hoop
        self.drone_last_side = [0] * len(hoop_poses)
        self.get_logger().info(f"Course set with {len(hoop_poses)} hoops")

    def update_scoring(self):
        """Check if drone passes through any hoop using directional detection. (COMPLETELY REWRITTEN)"""
        if not self.uav_position or not self.hoop_poses:
            return
        
        x, y, z = self.uav_position
        
        # Check each hoop for passage
        for i, hoop in enumerate(self.hoop_poses):
            if self.passed_hoops[i]:  # Skip if already passed
                continue
                
            hoop_x, hoop_y, hoop_z, roll, pitch, yaw = hoop
            
            # Get hoop radius from parameters
            hoop_radius = self.get_parameter('hoop_tolerance').get_parameter_value().double_value
            
            # NEW: Transform drone position to hoop local frame
            drone_pos_in_hoop = self._world_to_hoop_local(
                (x, y, z), 
                (hoop_y, hoop_x, hoop_z), #for some reason, we need to flip these 
                (roll, pitch, yaw)
            )
            
            # NEW: Check if drone is within hoop opening (cylindrical)
            inside_hoop = (abs(drone_pos_in_hoop[0]) <= hoop_radius and 
                          abs(drone_pos_in_hoop[2]) <= hoop_radius)  # x and z within radius
            
            if inside_hoop:
                # NEW: Determine which side of hoop drone is on
                current_side = 1 if drone_pos_in_hoop[1] > 0.0 else -1
                last_side = self.drone_last_side[i]
                
                # NEW: Check if drone crossed from one side to the other
                if last_side != 0 and last_side != current_side:
                    # Drone passed through the hoop!
                    self.passed_hoops[i] = True
                    curr = time.time()
                    #time decay
                    self.current_score += 10 * (0.99 ** (curr - self.start_time))
                    
                    # Publish status message (SAME AS BEFORE)
                    status_msg = String()
                    status_msg.data = f"Hoop {i+1} passed! Score: {self.current_score}"
                    self.status_publisher.publish(status_msg)
                    
                    # Publish score update (SAME AS BEFORE)
                    score_msg = Float32()
                    score_msg.data = float(self.current_score)
                    self.score_publisher.publish(score_msg)
                    
                    # Log the achievement
                    self.get_logger().info(f"🎯 HOOP {i+1} PASSED THROUGH! Score: {self.current_score}")
                    self.get_logger().info(f"Crossed from side {last_side} to side {current_side}")
                
                # Update side tracking
                self.drone_last_side[i] = current_side
            else:
                # Drone not in hoop, reset side tracking
                self.drone_last_side[i] = 0
        
        # Update hoop colors (throttled)
        current_time = time.time()
        if current_time - self.last_color_update_time >= self.color_update_interval:
            # Do initial setup on first update if not done yet
            if not self.initial_color_setup_done:
                # Wait a bit after startup for Gazebo to be ready
                if current_time - self.start_time >= 2.0:
                    self.initial_color_setup_done = True
                    self.get_logger().info("Initial hoop colors set (first hoop green, others red)")
            
            self.update_hoop_colors()
            self.last_color_update_time = current_time
        
        self.check_landing_bonus()
        # Throttled position logging (SAME AS BEFORE)
        if self.uav_position:
            self.get_logger().info(f"UAV Position: x={x:.2f}, y={y:.2f}, z={z:.2f}", throttle_duration_sec=5.0)

    def check_landing_bonus(self):
        """Check if drone has landed at origin and award bonus points."""
        if not self.uav_position or self.landing_bonus_awarded:
            return
        
        x, y, z = self.uav_position
        
        # Check if all hoops have been passed
        all_hoops_passed = any(self.passed_hoops)
        if not all_hoops_passed:
            return
        
        # Get landing parameters
        landing_tolerance = self.get_parameter('landing_tolerance').get_parameter_value().double_value
        landing_altitude_max = self.get_parameter('landing_altitude_max').get_parameter_value().double_value
        
        # Check if drone is near origin and low enough to be considered "landed"
        distance_from_origin = math.sqrt(x*x + y*y)
        is_near_origin = distance_from_origin <= landing_tolerance
        is_low_enough = z <= landing_altitude_max
        
        if is_near_origin and is_low_enough:
            self.landed_at_origin = True
            self.landing_bonus_awarded = True
            
            # Award landing bonus
            landing_bonus = self.get_parameter('landing_bonus').get_parameter_value().double_value
            self.current_score += landing_bonus
            
            # Publish status
            status_msg = String()
            status_msg.data = f"🏁 LANDING BONUS! +{landing_bonus} points! Total: {self.current_score:.1f}"
            self.status_publisher.publish(status_msg)
            
            # Publish score update
            score_msg = Float32()
            score_msg.data = float(self.current_score)
            self.score_publisher.publish(score_msg)
            
            # Log achievement
            self.get_logger().info(f"🏁 LANDING BONUS AWARDED! +{landing_bonus} points at origin (0,0)")
            self.get_logger().info(f"Final Score: {self.current_score:.1f}")
    def _world_to_hoop_local(self, 
                            world_pos: Tuple[float, float, float], 
                            hoop_pos: Tuple[float, float, float], 
                            hoop_euler: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """Convert world position to hoop local coordinates. (NEW METHOD)"""
        wx, wy, wz = world_pos
        hx, hy, hz = hoop_pos
        roll, pitch, yaw = hoop_euler
        
        # Translate to hoop origin
        dx = wx - hx
        dy = wy - hy
        dz = wz - hz
        
        # Convert Euler angles to rotation matrix
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        
        # Rotation matrix (ZYX order)
        R = [
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr]
        ]
        
        # Apply rotation (transpose for world-to-local)
        local_x = R[0][0]*dx + R[1][0]*dy + R[2][0]*dz
        local_y = R[0][1]*dx + R[1][1]*dy + R[2][1]*dz
        local_z = R[0][2]*dx + R[1][2]*dy + R[2][2]*dz
        
        return (local_x, local_y, local_z)
    
    def position_callback(self, msg: VehicleLocalPosition):
        """Callback for UAV position updates. (SAME AS BEFORE)"""
        z_up = -float(msg.z)
        x = float(msg.x)
        y = float(msg.y)
        self.prev_position = self.uav_position
        self.uav_position = (x, y, z_up)
        
        current_time = time.time()
        self.position_history.append((x, y, z_up, current_time))
        
        if len(self.position_history) > self.max_history:
            self.position_history.pop(0)
        
        drone_position = Float32MultiArray()
        drone_position.data = [x, y, z_up]
        self.drone_publisher.publish(drone_position)
    
    def get_next_hoop_index(self) -> Optional[int]:
        """
        Find the index of the next hoop that hasn't been passed yet.
        Returns None if all hoops have been passed.
        """
        if not self.passed_hoops or not self.hoop_poses:
            return None
        
        for i, passed in enumerate(self.passed_hoops):
            if not passed:
                return i
        return None  # All hoops passed
    
    def update_hoop_colors(self):
        """
        Update hoop colors: green for next hoop, red for all others.
        Publishes MaterialColor messages to Gazebo via ros_gz_bridge.
        """
        if not self.hoop_poses:
            return
        
        next_hoop_idx = self.get_next_hoop_index()
        
        # Define colors (RGBA values 0-1)
        green_color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)   # Green
        red_color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)     # Red
        
        # Update all hoops
        for i in range(len(self.hoop_poses)):
            hoop_num = i + 1  # Hoops are numbered starting from 1
            
            # Choose color: green if this is the next hoop, red otherwise
            if i == next_hoop_idx:
                color = green_color
            else:
                color = red_color
            
            # Create MaterialColor message
            msg = MaterialColor()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            
            # Entity name format: "hoop_{num}::hoop::visual" 
            # (model_name::link_name::visual_name)
            msg.entity = Entity()
            msg.entity.name = f"hoop_{hoop_num}::hoop::visual"
            msg.entity.type = Entity.VISUAL  # Entity type: VISUAL = 4
            msg.entity.id = 0  # ID not needed when using name
            
            # Set material colors
            msg.ambient = color
            msg.diffuse = color
            msg.specular = ColorRGBA(r=0.5, g=0.5, b=0.5, a=1.0)  # Gray specular
            msg.emissive = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)  # No emissive
            msg.shininess = 32.0
            
            # Apply to first matching entity
            msg.entity_match = MaterialColor.FIRST
            
            # Publish the message
            self.visual_config_publisher.publish(msg)
        
        if next_hoop_idx is not None:
            self.get_logger().debug(
                f"Updated hoop colors: hoop_{next_hoop_idx + 1} is green, others are red",
                throttle_duration_sec=2.0
            )
    
def main(args=None):
    """Main function for scoring node. (SAME AS BEFORE)"""
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