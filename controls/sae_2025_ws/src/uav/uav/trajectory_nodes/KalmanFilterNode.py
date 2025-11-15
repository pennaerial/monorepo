#kalman_filter_node.py

import rclpy
from rclpy.node import Node
import numpy as np
from uav_interfaces.msg import PoseEstimate, DroneState
from geometry_msgs.msg import Point, Vector3
from builtin_interfaces.msg import Time
import yaml
import os
from pathlib import Path
from typing import Optional
from collections import deque
import time
import matplotlib.pyplot as plt

class StateEstimationNode(Node):
    """
    ros2 node for drone state estimation using extended Kalman Filter 
    """

    def __init__(self):
        super().__init__('state_estimation_node')

        #declaring & loading params for tuning
        self.declare_parameter('process_noise_pos', 0.1)
        self.declare_parameter('process_noise_vel', 0.5)
        self.declare_parameter('measurement_noise_pos', 1.0)
        self.declare_parameter('measurement_noise_rot', 2.0)
        self.declare_parameter('update_rate', 30.0)
        self.declare_parameter('debug', False)
        self.declare_parameter('config_file', '')
        self.declare_parameter('log_performance', False)
        self.declare_parameter('visualize', False)

        self._load_parameters()
        
        # loading config file
        config_file = self.get_parameter('config_file').value
        if config_file:
            self._load_config_file(config_file)

        self.state_dim = 6
        self.measurement_dim = 6  
        
        # initializing state estimate
        self.state = np.zeros(self.state_dim)  # [x, y, z, vx, vy, vz]
        self.covariance = np.eye(self.state_dim) * 10.0  
        
        # process noise covariance matrix
        self.Q = self._build_process_noise_matrix()
        
        # measurement noise covariance matrix
        self.R = self._build_measurement_noise_matrix()
        
        # state transition matrix (w/ constant vel)
        self.F = np.eye(self.state_dim)
        dt = 1.0 / self.update_rate
        self.F[0, 3] = dt  # x += vx * dt
        self.F[1, 4] = dt  # y += vy * dt
        self.F[2, 5] = dt  # z += vz * dt
        
        # measurement matrix (using position for now)
        self.H = np.zeros((self.measurement_dim, self.state_dim))
        self.H[0, 0] = 1.0  # measure x
        self.H[1, 1] = 1.0  # measure y
        self.H[2, 2] = 1.0  # measure z
        # rotation doesn't directly effect velocity so i didn't use it here but matrix is big enough to add them
        
        # timing
        self.last_update_time = None
        self.last_measurement_time = None
        
        # logging performance
        self.log_performance = self.get_parameter('log_performance').value
        self.visualize = self.get_parameter('visualize').value
        if self.log_performance or self.visualize:
            self.state_history = deque(maxlen=1000)
            self.measurement_history = deque(maxlen=1000)
            self.time_history = deque(maxlen=1000)
        
        self.pose_subscription = self.create_subscription(
            PoseEstimate,
            '/pose_estimate',
            self.pose_callback,
            10
        )
        
        self.state_publisher = self.create_publisher(
            DroneState,
            '/drone_state',
            10
        )
        
        # timer for prediction step when no measurements arrive
        self.timer = self.create_timer(1.0 / self.update_rate, self.timer_callback)
        
        self.get_logger().info('State Estimation Node initialized')
        self.get_logger().info(f'Process noise (pos): {self.process_noise_pos}, (vel): {self.process_noise_vel}')
        self.get_logger().info(f'Measurement noise (pos): {self.measurement_noise_pos}, (rot): {self.measurement_noise_rot}')

    def _load_parameters(self):
        """load params"""
        self.process_noise_pos = self.get_parameter('process_noise_pos').value
        self.process_noise_vel = self.get_parameter('process_noise_vel').value
        self.measurement_noise_pos = self.get_parameter('measurement_noise_pos').value
        self.measurement_noise_rot = self.get_parameter('measurement_noise_rot').value
        self.update_rate = self.get_parameter('update_rate').value
        self.debug = self.get_parameter('debug').value

    def _load_config_file(self, config_path: str):
        """load params from YAML config file"""
        if not os.path.isabs(config_path):
            config_path = os.path.join(
                os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
                'config',
                config_path
            )
        
        if os.path.exists(config_path):
            try:
                with open(config_path, 'r') as f:
                    config = yaml.safe_load(f)
                    if 'kalman_filter' in config:
                        kf_config = config['kalman_filter']
                        self.process_noise_pos = kf_config.get('process_noise_pos', self.process_noise_pos)
                        self.process_noise_vel = kf_config.get('process_noise_vel', self.process_noise_vel)
                        self.measurement_noise_pos = kf_config.get('measurement_noise_pos', self.measurement_noise_pos)
                        self.measurement_noise_rot = kf_config.get('measurement_noise_rot', self.measurement_noise_rot)
                        self.update_rate = kf_config.get('update_rate', self.update_rate)
                        self.get_logger().info(f'Loaded config from {config_path}')
            except Exception as e:
                self.get_logger().warn(f'Failed to load config file: {e}')
        else:
            self.get_logger().warn(f'Config file not found: {config_path}')

    
    def _build_process_noise_matrix(self) -> np.ndarray:
        return np.diag([self.process_noise_pos] * 3 + [self.process_noise_vel] * 3)

    def _build_measurement_noise_matrix(self) -> np.ndarray:
        R_pos = np.eye(3) * self.measurement_noise_pos
        R_rot = np.eye(3) * self.measurement_noise_rot
        return np.diag([self.measurement_noise_pos] * 3 + [self.measurement_noise_rot] * 3)

    def _compute_dt(self) -> float:
        """compute time since last update"""
        current_time = time.time()
        if self.last_update_time is None:
            self.last_update_time = current_time
            return 1.0 / self.update_rate
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        return max(dt, 1e-6)  

    def predict(self, dt: float):
        """
        prediction step w/ dt param
        
        """
        # update state transition matrix with current dt
        F = np.eye(self.state_dim)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        
        # predict state: x_pred = F * x
        self.state = F @ self.state
        
        # predict covariance: P_pred = F * P * F^T + Q
        self.covariance = F @ self.covariance @ F.T + self.Q
        
        if self.debug:
            self.get_logger().debug(f'Predicted state: {self.state}')

    def update(self, measurement: np.ndarray, measurement_cov: np.ndarray):
        """
        updating step of EKF w/ measurement vector & measurement covariance matrix as params
        
        """
        # get position measurement
        z_pos = measurement[:3]
        
        # find innovation (for prediction): y = z - H * x
        H_pos = self.H[:3, :]  
        innovation = z_pos - H_pos @ self.state
        
        # find innovation covariance: S = H * P * H^T + R
        R_pos = measurement_cov[:3, :3]  
        S = H_pos @ self.covariance @ H_pos.T + R_pos
        
        # find kalman gain: K = P * H^T * S^-1
        K = self.covariance @ H_pos.T @ np.linalg.inv(S)
        
        # update state: x = x + K * y
        self.state = self.state + K @ innovation
        
        # update covariance: P = (I - K * H) * P
        I = np.eye(self.state_dim)
        self.covariance = (I - K @ H_pos) @ self.covariance
        
        if self.debug:
            self.get_logger().debug(f'Updated state: {self.state}')
            self.get_logger().debug(f'Innovation: {innovation}')

    def pose_callback(self, msg: PoseEstimate):
        # get measurements
        position = msg.position
        rotation = msg.rotation
        
        measurement = np.array([
            position.x,
            position.y,
            position.z,
            rotation.x,
            rotation.y,
            rotation.z
        ])
        
        # get covariance matrix
        if len(msg.covariance) == 36:
            measurement_cov = np.array(msg.covariance).reshape(6, 6)
        else:
            # default covariance if not provided
            measurement_cov = self.R
        
        # get delta time
        dt = self._compute_dt()
        
        # predict step
        self.predict(dt)
        
        # update step
        self.update(measurement, measurement_cov)
        
        # publish state estimate
        self._publish_state()
        
        # log performance (if enabled)
        if self.log_performance or self.visualize:
            self.state_history.append(self.state.copy())
            self.measurement_history.append(measurement.copy())
            self.time_history.append(time.time())
        
        self.last_measurement_time = time.time()

    def timer_callback(self):
        """timer callback for prediction-only updates when there are no measurements"""
        dt = self._compute_dt()
        
        # only predict if no recent measurements (more than 2 update periods)
        if (self.last_measurement_time is None or 
            time.time() - self.last_measurement_time > 2.0 / self.update_rate):
            self.predict(dt)
            self._publish_state()

    def _publish_state(self):
        """publishing current state estimate"""
        msg = DroneState()
        
        # position
        msg.position = Point()
        msg.position.x = float(self.state[0])
        msg.position.y = float(self.state[1])
        msg.position.z = float(self.state[2])
        
        # velocity
        msg.velocity = Vector3()
        msg.velocity.x = float(self.state[3])
        msg.velocity.y = float(self.state[4])
        msg.velocity.z = float(self.state[5])
        
        # covariance (flattening 6x6 matrix)
        msg.covariance = self.covariance.flatten().tolist()
        
        # timestamp
        now = self.get_clock().now()
        msg.timestamp = now.to_msg()
        
        self.state_publisher.publish(msg)

    def save_performance_log(self, filename: Optional[str] = None):
        """save performance logs to file"""
        if not self.log_performance or len(self.state_history) == 0:
            return
        
        if filename is None:
            log_dir = Path.home() / 'kalman_filter_logs'
            log_dir.mkdir(exist_ok=True)
            filename = log_dir / f'state_estimation_{int(time.time())}.npz'
        
        np.savez(
            filename,
            state_history=np.array(self.state_history),
            measurement_history=np.array(self.measurement_history),
            time_history=np.array(self.time_history)
        )
        self.get_logger().info(f'Saved performance log to {filename}')

    def visualize_state_traces(self):
        """visualize state traces using matplotlib"""
        if not self.visualize or len(self.state_history) == 0:
            return
        
        if not MATPLOTLIB_AVAILABLE:
            self.get_logger().warn('Matplotlib not available, skipping visualization')
            return
        
        states = np.array(self.state_history)
        measurements = np.array(self.measurement_history)
        times = np.array(self.time_history)
        
        if len(times) > 0:
            times = times - times[0]  
        
        fig, axes = plt.subplots(3, 2, figsize=(12, 10))
        fig.suptitle('Kalman Filter State Estimation Traces')
        
        labels = ['x', 'y', 'z']
        for i in range(3):
            # position
            axes[i, 0].plot(times, states[:, i], 'b-', label='Estimated', linewidth=2)
            axes[i, 0].plot(times, measurements[:, i], 'r--', label='Measurement', alpha=0.7)
            axes[i, 0].set_xlabel('Time (s)')
            axes[i, 0].set_ylabel(f'{labels[i]} Position (m)')
            axes[i, 0].legend()
            axes[i, 0].grid(True)
            
            # velocity
            axes[i, 1].plot(times, states[:, i+3], 'g-', label='Estimated', linewidth=2)
            axes[i, 1].set_xlabel('Time (s)')
            axes[i, 1].set_ylabel(f'{labels[i]} Velocity (m/s)')
            axes[i, 1].legend()
            axes[i, 1].grid(True)
        
        plt.tight_layout()
        
        # save figure
        log_dir = Path.home() / 'kalman_filter_logs'
        log_dir.mkdir(exist_ok=True)
        fig_path = log_dir / f'state_traces_{int(time.time())}.png'
        plt.savefig(fig_path, dpi=150)
        self.get_logger().info(f'Saved visualization to {fig_path}')
        
        plt.close()


def main(args=None):
    rclpy.init(args=args)
    node = StateEstimationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
        # save logs & visualize before shut down (should help w debugging)
        if node.log_performance:
            node.save_performance_log()
        if node.visualize:
            node.visualize_state_traces()
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
