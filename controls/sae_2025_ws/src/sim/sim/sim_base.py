#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude
from geometry_msgs.msg import Point
from typing import Optional, Tuple, Dict, Any
import threading
import time
import numpy as np


class SimBase(Node):
    """
    Base simulation class providing UAV position tracking and competition management.
    """
    
    def __init__(self, node_name: str = 'sim_base'):
        super().__init__(node_name)
        
        # UAV position tracking
        self.uav_position: Optional[Tuple[float, float, float]] = None
        self.position_lock = threading.Lock()
        
        # UAV attitude tracking
        self.uav_attitude: Optional[Tuple[float, float, float]] = None  # roll, pitch, yaw
        self.attitude_lock = threading.Lock()
        
        # Position polling timer (10Hz)
        self.create_timer(0.1, self.poll_uav_position)
        
        # Subscribe to UAV position
        self.position_subscription = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.position_callback,
            10
        )
        
        # Subscribe to UAV attitude
        self.attitude_subscription = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            10
        )
        
        self.get_logger().info(f"SimBase initialized with node name: {node_name}")
    
    def position_callback(self, msg: VehicleLocalPosition):
        """Callback for UAV position updates."""
        with self.position_lock:
            self.uav_position = (msg.x, msg.y, msg.z)
    
    def poll_uav_position(self):
        """Periodic UAV position polling (10Hz)."""
        with self.position_lock:
            if self.uav_position is not None:
                x, y, z = self.uav_position
                self.get_logger().debug(f"UAV Position: x={x:.2f}, y={y:.2f}, z={z:.2f}")
        
        with self.attitude_lock:
            if self.uav_attitude is not None:
                roll, pitch, yaw = self.uav_attitude
                self.get_logger().debug(f"UAV Attitude: roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}")
    
    def get_uav_position(self) -> Optional[Tuple[float, float, float]]:
        """Get current UAV position thread-safely."""
        with self.position_lock:
            return self.uav_position
    
    def get_uav_attitude(self) -> Optional[Tuple[float, float, float]]:
        """Get current UAV attitude (roll, pitch, yaw) thread-safely."""
        with self.attitude_lock:
            return self.uav_attitude
    
    def wait_for_uav_position(self, timeout: float = 10.0) -> bool:
        """Wait for UAV position to be available."""
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.get_uav_position() is not None:
                return True
            time.sleep(0.1)
        return False
    
    def wait_for_uav_attitude(self, timeout: float = 10.0) -> bool:
        """Wait for UAV attitude to be available."""
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.get_uav_attitude() is not None:
                return True
            time.sleep(0.1)
        return False
    
    def load_competition_config(self, config: Dict[str, Any]) -> Dict[str, Any]:
        """Load and validate competition configuration."""
        required_keys = ['dlz', 'uav', 'num_hoops', 'max_dist']
        
        for key in required_keys:
            if key not in config:
                raise ValueError(f"Missing required config key: {key}")
        
        # Validate coordinate tuples
        for key in ['dlz', 'uav']:
            if not isinstance(config[key], (list, tuple)) or len(config[key]) != 3:
                raise ValueError(f"Config key '{key}' must be a 3-element list/tuple")
        
        self.get_logger().info(f"Competition config loaded: {config}")
        return config
    
    def attitude_callback(self, msg: VehicleAttitude):
        """Callback for UAV attitude updates."""
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        q = msg.q  # [qx, qy, qz, qw]
        
        # Convert quaternion to Euler angles
        roll = np.arctan2(2.0 * (q[3] * q[0] + q[1] * q[2]), 
                         1.0 - 2.0 * (q[0]**2 + q[1]**2))
        
        pitch = np.arcsin(2.0 * (q[3] * q[1] - q[2] * q[0]))
        
        yaw = np.arctan2(2.0 * (q[3] * q[2] + q[0] * q[1]), 
                       1.0 - 2.0 * (q[1]**2 + q[2]**2))
        
        with self.attitude_lock:
            self.uav_attitude = (roll, pitch, yaw)


def main(args=None):
    """Test the SimBase class."""
    rclpy.init(args=args)
    
    sim_base = SimBase()
    
    try:
        rclpy.spin(sim_base)
    except KeyboardInterrupt:
        pass
    finally:
        sim_base.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()