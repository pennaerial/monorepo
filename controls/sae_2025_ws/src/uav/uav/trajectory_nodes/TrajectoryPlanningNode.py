#!/usr/bin/env python3
"""
Trajectory Planning Node for UAV Hoop Navigation

Subscribes to:
- /drone_state (DroneState.msg) - Current drone state from Kalman filter
- /hoop_geometries (HoopGeometry.msg[]) - Hoop geometries from mapping node
- /hoop_pose (optional) - Target hoop pose from pose estimation

Publishes:
- /trajectory (Trajectory.msg) - Generated trajectory
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
from typing import Optional, List
import math

# Message imports
from uav_interfaces.msg import DroneState, Trajectory, HoopGeometry
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Header

# Planner import
from uav.trajectory import PolynomialTrajectoryPlanner, Trajectory as PlannerTrajectory


class TrajectoryPlanningNode(Node):
    """
    ROS2 node for generating smooth polynomial trajectories to hoop centers.
    """
    
    def __init__(self):
        super().__init__('trajectory_planning_node')
        
        # Declare parameters
        self.declare_parameter('polynomial_order', 7)
        self.declare_parameter('max_velocity', 2.0)
        self.declare_parameter('max_acceleration', 1.0)
        self.declare_parameter('max_jerk', 2.0)
        self.declare_parameter('duration_factor', 1.2)
        self.declare_parameter('min_duration', 1.0)
        self.declare_parameter('max_duration', 30.0)
        self.declare_parameter('replan_frequency', 10.0)  # Hz
        self.declare_parameter('safety_margin', 0.5)  # meters
        self.declare_parameter('waypoint_segment_length', 0.5)  # meters
        
        # Get parameters
        polynomial_order = self.get_parameter('polynomial_order').value
        max_velocity = self.get_parameter('max_velocity').value
        max_acceleration = self.get_parameter('max_acceleration').value
        max_jerk = self.get_parameter('max_jerk').value
        duration_factor = self.get_parameter('duration_factor').value
        min_duration = self.get_parameter('min_duration').value
        max_duration = self.get_parameter('max_duration').value
        self.replan_frequency = self.get_parameter('replan_frequency').value
        self.safety_margin = self.get_parameter('safety_margin').value
        
        # Initialize planner
        self.planner = PolynomialTrajectoryPlanner(
            polynomial_order=polynomial_order,
            minimize_derivative=4,  # Snap minimization
            max_velocity=max_velocity,
            max_acceleration=max_acceleration,
            max_jerk=max_jerk,
            duration_factor=duration_factor,
            min_duration=min_duration,
            max_duration=max_duration
        )
        
        # State variables
        self.current_drone_state: Optional[DroneState] = None
        self.target_hoop: Optional[HoopGeometry] = None
        self.hoop_geometries: List[HoopGeometry] = []
        self.current_trajectory: Optional[PlannerTrajectory] = None
        self.trajectory_start_time: Optional[float] = None
        self.last_replan_time: float = 0.0
        
        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.drone_state_sub = self.create_subscription(
            DroneState,
            '/drone_state',
            self.drone_state_callback,
            qos_profile
        )
        
        self.hoop_geometries_sub = self.create_subscription(
            HoopGeometry,
            '/hoop_geometries',
            self.hoop_geometries_callback,
            qos_profile
        )
        
        # Publisher
        self.trajectory_pub = self.create_publisher(
            Trajectory,
            '/trajectory',
            qos_profile
        )
        
        # Replanning timer
        self.replan_timer = self.create_timer(
            1.0 / self.replan_frequency,
            self.replan_callback
        )
        
        self.get_logger().info(
            f'TrajectoryPlanningNode initialized with '
            f'polynomial_order={polynomial_order}, '
            f'max_velocity={max_velocity} m/s, '
            f'max_acceleration={max_acceleration} m/s²'
        )
    
    def drone_state_callback(self, msg: DroneState):
        """Callback for drone state updates."""
        self.current_drone_state = msg
        
        # Trigger replanning if we have all necessary data
        if self.should_replan():
            self.plan_trajectory()
    
    def hoop_geometries_callback(self, msg: HoopGeometry):
        """Callback for hoop geometry updates."""
        # Update or add hoop to list
        hoop_id = msg.hoop_id
        
        # Remove existing hoop with same ID
        self.hoop_geometries = [h for h in self.hoop_geometries if h.hoop_id != hoop_id]
        
        # Add new hoop
        if not msg.passed:  # Only plan to hoops that haven't been passed
            self.hoop_geometries.append(msg)
            
            # Set as target if it's the first/nearest hoop
            if self.target_hoop is None:
                self.select_target_hoop()
        
        # Trigger replanning
        if self.should_replan():
            self.plan_trajectory()
    

    def select_target_hoop(self):
        """Select the nearest hoop as target."""
        if not self.hoop_geometries or self.current_drone_state is None:
            self.target_hoop = None
            return
        
        if not self.current_drone_state.valid:
            self.target_hoop = None
            return
        
        # Find nearest hoop that hasn't been passed
        current_pos = np.array(self.current_drone_state.position)
        min_distance = float('inf')
        nearest_hoop = None
        
        for hoop in self.hoop_geometries:
            if hoop.passed:
                continue
            
            hoop_pos = np.array(hoop.center_position)
            distance = np.linalg.norm(hoop_pos - current_pos)
            
            if distance < min_distance:
                min_distance = distance
                nearest_hoop = hoop
        
        self.target_hoop = nearest_hoop
        
        if self.target_hoop:
            self.get_logger().info(
                f'Selected target hoop: ID={self.target_hoop.hoop_id}, '
                f'distance={min_distance:.2f}m'
            )
    
    def should_replan(self) -> bool:
        """Check if replanning is needed."""
        if self.current_drone_state is None or not self.current_drone_state.valid:
            return False
        
        if self.target_hoop is None:
            return False
        
        return True
    


    def plan_trajectory(self):
        """Generate a new trajectory to the target hoop."""
        if not self.should_replan():
            return
        
        # Get current state
        current_pos = np.array(self.current_drone_state.position)
        current_vel = np.array(self.current_drone_state.velocity)
        current_acc = np.array(self.current_drone_state.acceleration)
        
        # Get target position (hoop center)
        target_pos = np.array(self.target_hoop.center_position)
        
        # Convert to lists for planner
        start_pos = current_pos.tolist()
        end_pos = target_pos.tolist()
        start_vel = current_vel.tolist()
        start_acc = current_acc.tolist()
        end_vel = [0.0, 0.0, 0.0]  # Stop at hoop center
        end_acc = [0.0, 0.0, 0.0]

        # Keep track of the attempts to generate a trajectory
        trajectory_attempts = 0
        max_attempts = 5


        while True:
            try:
                # Generate trajectory
                trajectory = self.planner.generate_trajectory(
                    start_pos=start_pos,
                    end_pos=end_pos,
                    start_vel=start_vel,
                    start_acc=start_acc,
                    end_vel=end_vel,
                    end_acc=end_acc
                )
                
                # Check for collisions
                collision_geometries = [h for h in self.hoop_geometries if h.hoop_id != self.target_hoop.hoop_id]
                has_collision = self.planner.check_collision(
                    trajectory,
                    collision_geometries,
                    safety_margin=self.safety_margin
                )
                
                if has_collision:
                    self.get_logger().warn('Generated trajectory has collisions, attempting to replan...')
                    # TODO: Implement collision avoidance (add intermediate waypoints)
                    # For now, we'll still publish but log a warning
                
                # Validate constraints
                is_valid, violations = self.planner.validate_trajectory_constraints(trajectory)
                if not is_valid:
                    self.get_logger().warn('Trajectory constraint violations detected:')
                    for violation in violations[:5]:  # Log first 5 violations
                        self.get_logger().warn(f'  {violation}')
                else:
                    self.get_logger().info(
                        f'Generated valid trajectory: duration={trajectory.total_duration:.2f}s, '
                        f'waypoints={len(trajectory.waypoints)}'
                    )
                
                # Store trajectory
                self.current_trajectory = trajectory
                # Store start time as seconds since node start
                self.trajectory_start_time = self.get_clock().now().nanoseconds / 1e9
                
                # Publish trajectory
                self.publish_trajectory(trajectory)



                # If we have gotten a valid trajectory that doesn't have collisions, or we have tried too many times, break out of the loop
                if (not has_collision or is_valid or trajectory_attempts >= max_attempts):
                    break

            except Exception as e:
                self.get_logger().error(f'Error generating trajectory: {e}')
                import traceback
                self.get_logger().error(traceback.format_exc())
                break # Break out of the loop if we fail to generate a trajectory
        


    def publish_trajectory(self, trajectory: PlannerTrajectory):
        """Publish trajectory as ROS2 message."""
        msg = Trajectory()
        
        # Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ned'
        
        # Trajectory metadata
        msg.duration = float(trajectory.total_duration)
        msg.polynomial_order = trajectory.polynomial_order
        msg.valid = True
        
        # Sample trajectory at higher frequency for smoother interpolation
        # Sample at 20 Hz (every 0.05 seconds) for good interpolation
        sample_rate = 20.0  # Hz
        dt = 1.0 / sample_rate
        sample_times = np.arange(0.0, trajectory.total_duration + dt, dt)
        sample_times = np.clip(sample_times, 0.0, trajectory.total_duration)
        
        msg.waypoint_times = [float(t) for t in sample_times]
        msg.waypoints = []
        msg.velocities = []
        msg.accelerations = []
        msg.yaws = []
        
        # Sample trajectory at each time
        for i, t in enumerate(sample_times):
            pos, vel, acc, _ = self.planner.evaluate_trajectory(trajectory, t)
            
            # Position
            point = Point()
            point.x = float(pos[0])
            point.y = float(pos[1])
            point.z = float(pos[2])
            msg.waypoints.append(point)
            
            # Velocity
            velocity = Vector3()
            velocity.x = float(vel[0])
            velocity.y = float(vel[1])
            velocity.z = float(vel[2])
            msg.velocities.append(velocity)
            
            # Acceleration
            acceleration = Vector3()
            acceleration.x = float(acc[0])
            acceleration.y = float(acc[1])
            acceleration.z = float(acc[2])
            msg.accelerations.append(acceleration)
            
            # Yaw (point in direction of velocity)
            vel_mag = np.linalg.norm(vel[:2])
            if vel_mag > 0.1:
                yaw = math.atan2(vel[1], vel[0])
            elif i < len(sample_times) - 1:
                # Use direction to next waypoint if velocity is small
                next_pos, _, _, _ = self.planner.evaluate_trajectory(trajectory, sample_times[i + 1])
                dx = next_pos[0] - pos[0]
                dy = next_pos[1] - pos[1]
                yaw = math.atan2(dy, dx) if np.linalg.norm([dx, dy]) > 0.1 else 0.0
            else:
                yaw = 0.0
            msg.yaws.append(float(yaw))
        
        # Publish
        self.trajectory_pub.publish(msg)
        self.get_logger().info(
            f'Published trajectory: duration={msg.duration:.2f}s, '
            f'waypoints={len(msg.waypoints)}, '
            f'sample_rate={sample_rate}Hz'
        )
    
    def replan_callback(self):
        """Periodic callback for replanning."""
        current_time = self.get_clock().now().nanoseconds / 1e9
        time_since_replan = current_time - self.last_replan_time
        
        # Check if we need to replan
        if self.should_replan():
            # Select target hoop (in case it changed)
            self.select_target_hoop()
            
            # Replan if:
            # 1. No trajectory exists
            # 2. Trajectory is nearly complete (>90% done)
            # 3. Significant state change detected
            if self.current_trajectory is None:
                self.plan_trajectory()
            elif self.trajectory_start_time is not None:
                elapsed_time = current_time - self.trajectory_start_time
                progress = elapsed_time / self.current_trajectory.total_duration if self.current_trajectory.total_duration > 0 else 0.0
                
                if progress > 0.9:  # 90% complete
                    self.get_logger().info('Trajectory nearly complete, replanning...')
                    self.plan_trajectory()
            
            self.last_replan_time = current_time



def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlanningNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down TrajectoryPlanningNode')
    except Exception as e:
        node.get_logger().error(f'Error in TrajectoryPlanningNode: {e}')
        import traceback
        node.get_logger().error(traceback.format_exc())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

