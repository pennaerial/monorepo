#!/usr/bin/env python3
"""
Simplified Trajectory Planning Node for UAV Hoop Navigation

Subscribes to:
- /drone_state (DroneState.msg) - Current drone state from Kalman filter
- /hoop_geometries (HoopGeometry.msg) - Hoop geometries from mapping node

Publishes:
- /trajectory_setpoint (TrajectorySetpoint.msg) - Current position and velocity setpoint
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
from typing import Optional, List

# Message imports
from uav_interfaces.msg import DroneState, HoopGeometry
from px4_msgs.msg import TrajectorySetpoint

# Planner import
from uav.cv.polynomial_planner import PolynomialTrajectoryPlanner, Trajectory as PlannerTrajectory


class TrajectoryPlanningNode(Node):
    """
    Simplified ROS2 node for generating smooth polynomial trajectories to hoop centers.
    Evaluates trajectory at current time and publishes position + velocity setpoints.
    """
    
    def __init__(self):
        super().__init__('trajectory_planning_node')
        
        # Declare parameters
        self.declare_parameter('polynomial_order', 7)
        self.declare_parameter('max_velocity', 2.0)
        self.declare_parameter('max_acceleration', 1.0)
        self.declare_parameter('replan_frequency', 10.0)  # Hz - how often to replan
        self.declare_parameter('publish_frequency', 50.0)  # Hz - how often to publish setpoints
        
        # Get parameters
        polynomial_order = self.get_parameter('polynomial_order').value
        max_velocity = self.get_parameter('max_velocity').value
        max_acceleration = self.get_parameter('max_acceleration').value
        self.replan_frequency = self.get_parameter('replan_frequency').value
        self.publish_frequency = self.get_parameter('publish_frequency').value
        
        # Initialize planner
        self.planner = PolynomialTrajectoryPlanner(
            polynomial_order=polynomial_order,
            max_velocity=max_velocity,
            max_acceleration=max_acceleration
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
        
        # Publisher - publish TrajectorySetpoint with position and velocity
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/trajectory_setpoint',
            qos_profile
        )
        
        # Replanning timer - replan trajectory periodically
        self.replan_timer = self.create_timer(
            1.0 / self.replan_frequency,
            self.replan_callback
        )
        
        # Publishing timer - publish current setpoint at high frequency
        self.publish_timer = self.create_timer(
            1.0 / self.publish_frequency,
            self.publish_setpoint_callback
        )
        
        self.get_logger().info(
            f'TrajectoryPlanningNode initialized with '
            f'polynomial_order={polynomial_order}, '
            f'max_velocity={max_velocity} m/s, '
            f'max_acceleration={max_acceleration} m/s², '
            f'publish_frequency={self.publish_frequency} Hz'
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
        
        try:
            # Get current state
            current_pos = np.array(self.current_drone_state.position)
            current_vel = np.array(self.current_drone_state.velocity)
            current_acc = np.array(self.current_drone_state.acceleration)
            
            # Get target position (hoop center)
            target_pos = np.array(self.target_hoop.center_position)
            
            # Generate trajectory
            trajectory = self.planner.generate_trajectory(
                start_pos=current_pos.tolist(),
                end_pos=target_pos.tolist(),
                start_vel=current_vel.tolist(),
                end_vel=[0.0, 0.0, 0.0],  # Stop at hoop center
                start_acc=current_acc.tolist(),
                end_acc=[0.0, 0.0, 0.0]
            )
            
            # Store trajectory
            self.current_trajectory = trajectory
            # Store start time as seconds since node start
            self.trajectory_start_time = self.get_clock().now().nanoseconds / 1e9
            
            self.get_logger().info(
                f'Generated trajectory: duration={trajectory.total_duration:.2f}s'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error generating trajectory: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def publish_setpoint_callback(self):
        """High-frequency callback to publish current position and velocity setpoint."""
        if self.current_trajectory is None or self.trajectory_start_time is None:
            return
        
        if self.current_drone_state is None or not self.current_drone_state.valid:
            return
        
        try:
            # Calculate elapsed time since trajectory start
            # Note: This is just for evaluating the polynomial - we don't care about
            # hitting a specific time. The trajectory is just a smooth path.
            current_time = self.get_clock().now().nanoseconds / 1e9
            elapsed_time = current_time - self.trajectory_start_time
            
            # Evaluate trajectory at current time
            # If elapsed_time > duration, we'll just get the end position (clamped)
            pos, vel = self.planner.evaluate_trajectory(self.current_trajectory, elapsed_time)
            
            # Check if we're close to target - if so, we can replan or stop
            if self.target_hoop is not None:
                target_pos = np.array(self.target_hoop.center_position)
                current_pos = np.array(self.current_drone_state.position)
                distance_to_target = np.linalg.norm(target_pos - current_pos)
                
                # If very close to target, just publish target position
                if distance_to_target < 0.1:  # 10cm threshold
                    pos = target_pos
                    vel = np.array([0.0, 0.0, 0.0])
            
            # Create and publish TrajectorySetpoint message
            msg = TrajectorySetpoint()
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)  # PX4 uses microseconds
            msg.position = [float(pos[0]), float(pos[1]), float(pos[2])]
            msg.velocity = [float(vel[0]), float(vel[1]), float(vel[2])]
            # Set acceleration and yaw to NaN (let PX4 handle it)
            msg.acceleration = [float('nan'), float('nan'), float('nan')]
            msg.yaw = float('nan')
            msg.yawspeed = float('nan')
            
            self.trajectory_setpoint_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing setpoint: {e}')
    
    def replan_callback(self):
        """Periodic callback for replanning."""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Check if we need to replan
        if self.should_replan():
            # Select target hoop (in case it changed)
            self.select_target_hoop()
            
            # Replan if:
            # 1. No trajectory exists
            # 2. We're close to target (replan to next target or refine)
            # 3. We've been following this trajectory for a while (periodic replan)
            if self.current_trajectory is None:
                self.plan_trajectory()
            elif self.current_drone_state is not None and self.target_hoop is not None:
                # Check distance to target instead of time progress
                current_pos = np.array(self.current_drone_state.position)
                target_pos = np.array(self.target_hoop.center_position)
                distance_to_target = np.linalg.norm(target_pos - current_pos)
                
                # If close to target, replan (might be a new target or refinement)
                if distance_to_target < 0.5:  # 50cm threshold
                    self.get_logger().info(f'Close to target (distance={distance_to_target:.2f}m), replanning...')
                    self.plan_trajectory()
                # Or if we've been following this trajectory for too long, replan
                elif self.trajectory_start_time is not None:
                    elapsed_time = current_time - self.trajectory_start_time
                    # Replan if we've been following for longer than expected duration
                    # (means we're moving slower than expected)
                    if elapsed_time > self.current_trajectory.total_duration * 2.0:
                        self.get_logger().info('Trajectory taking longer than expected, replanning...')
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