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
from sim_interfaces.srv import HoopList
from sim.scoring import ScoringNode

class HoopScoringNode(ScoringNode):
    """
    Scoring node for in-house competition.
    Tracks UAV position and scores hoop passages using directional detection.
    """
    
    def __init__(self):
        super().__init__()
        
        # Declare parameters
        self.declare_parameter('hoop_tolerance', 1.5) 
        self.declare_parameter('competition_type', 'in_house')
        self.declare_parameter('competition_name', 'test')
        self.declare_parameter('course_type', 'straight')
        
        # Store parameters
        self.hoop_tolerance = hoop_tolerance
        self.landing_bonus = landing_bonus
        self.landing_tolerance = landing_tolerance
        self.landing_altitude_max = landing_altitude_max
        
        # Course data
        self.hoop_poses: List[Tuple[float, float, float, float, float, float]] = []
        self.passed_hoops: List[bool] = []
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
        
        # Additional publisher for drone position
        self.drone_publisher = self.create_publisher(Float32MultiArray, '/scoring/in_house/drone_position', 10)
        
        # Subscriber for UAV position
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
        
        # self.load_hoop_positions()
        # self.get_logger().info("Scoring node initialized.")
        self.hoop_client = self.create_client(HoopList, "list_hoops")
        self.hoops_loaded = False
        self.create_timer(0.5, self._try_request_hoops)

    def _try_request_hoops(self):
        if self.hoops_loaded:
            return

        if not self.hoop_client.wait_for_service(timeout_sec=0.0):
            self.get_logger().warn("Waiting for 'list_hoops' service...", throttle_duration_sec=5.0)
            return

        future = self.hoop_client.call_async(HoopList.Request())
        future.add_done_callback(self._on_hoops_response)
        self.hoops_loaded = True  # prevent repeated calls

    def _on_hoops_response(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"'list_hoops' failed: {e}")
            self.hoops_loaded = False  # try again later
            return

        self.hoop_poses = []
        self.passed_hoops = []
        self.drone_last_side = []
        for hoop in response.hoop_positions:
            self.hoop_poses.append((hoop.x, hoop.y, hoop.z, hoop.roll, hoop.pitch, hoop.yaw))
            self.passed_hoops.append(False)
            self.drone_last_side.append(0)

        self.get_logger().info(f"Loaded {len(self.hoop_poses)} hoops from 'list_hoops'")
    
    def load_hoop_positions(self):
        """Load hoop positions from ROS2 parameters."""
        try:
            client = self.create_client(HoopList, "list_hoops")
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn("Waiting for 'list_hoops' service...")

            req = HoopList.Request()  # empty request
            future = client.call_async(req)
            # Block here until the service returns, before main spin()
            # rclpy.spin_until_future_complete(self, future)
            # rclpy.spin_until_future_complete(self, future)

            if future.result() is None:
                self.get_logger().error(
                    f"'list_hoops' failed with exception: {future.exception()}"
                )
                raise RuntimeError("Service call to 'list_hoops' failed")

            response = future.result()

            # Expecting response.hoops to be a list of HoopPose messages
            self.hoop_poses = []
            self.passed_hoops = []
            self.drone_last_side = []

            for i, hoop in enumerate(response.hoop_positions):
                x = float(hoop.x)
                y = float(hoop.y)
                z = float(hoop.z)
                roll = float(hoop.roll)
                pitch = float(hoop.pitch)
                yaw = float(hoop.yaw)

                self.hoop_poses.append((x, y, z, roll, pitch, yaw))
                self.passed_hoops.append(False)
                self.drone_last_side.append(0)

                self.get_logger().info(
                    f"Hoop {i+1}: x={x:.2f}, y={y:.2f}, z={z:.2f}, "
                    f"roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}"
                )

            self.get_logger().info(f"Loaded {len(self.hoop_poses)} hoops from 'list_hoops' service.")

        except Exception as e:
            self.get_logger().error(f"Failed to load hoop positions: {e}")
            raise
        
    def set_course_hoops(self, hoop_poses: List[Tuple[float, float, float, float, float, float]]):
        """Set the hoop positions for scoring. (UPDATED)"""
        self.hoop_poses = hoop_poses
        self.passed_hoops = [False] * len(hoop_poses)
        # NEW: Initialize side tracking for each hoop
        self.drone_last_side = [0] * len(hoop_poses)
        self.get_logger().info(f"Course set with {len(hoop_poses)} hoops")

    def update_scoring(self) -> None:
        """
        Check if drone passes through any hoop using directional detection.
        
        Implements the abstract method from ScoringNode.
        """
        if not self.uav_position or not self.hoop_poses:
            return
        
        x, y, z = self.uav_position
        # Check each hoop for passage
        for i, hoop in enumerate(self.hoop_poses):
            if self.passed_hoops[i]:  # Skip if already passed
                continue
                
            hoop_x, hoop_y, hoop_z, roll, pitch, yaw = hoop
            
            # Use hoop tolerance from initialization
            hoop_radius = self.hoop_tolerance
            
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
                    # Time decay scoring
                    points = 10 * (0.99 ** (curr - self.start_time))
                    self.current_score += points
                    
                    # Publish status and score
                    self.publish_status(f"Hoop {i+1} passed! Score: {self.current_score}")
                    self.publish_score()
                    
                    # Log the achievement
                    self.get_logger().info(f"🎯 HOOP {i+1} PASSED THROUGH! Score: {self.current_score}")
                    self.get_logger().info(f"Crossed from side {last_side} to side {current_side}")
                
                # Update side tracking
                self.drone_last_side[i] = current_side
            else:
                # Drone not in hoop, reset side tracking
                self.drone_last_side[i] = 0
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
        
        # Check if drone is near origin and low enough to be considered "landed"
        distance_from_origin = math.sqrt(x*x + y*y)
        is_near_origin = distance_from_origin <= self.landing_tolerance
        is_low_enough = z <= self.landing_altitude_max
        
        if is_near_origin and is_low_enough:
            self.landed_at_origin = True
            self.landing_bonus_awarded = True
            
            # Award landing bonus
            self.current_score += self.landing_bonus
            
            # Publish status and score
            self.publish_status(f"🏁 LANDING BONUS! +{self.landing_bonus} points! Total: {self.current_score:.1f}")
            self.publish_score()
            
            # Log achievement
            self.get_logger().info(f"🏁 LANDING BONUS AWARDED! +{self.landing_bonus} points at origin (0,0)")
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