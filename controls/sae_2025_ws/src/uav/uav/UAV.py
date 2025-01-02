import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleCommand, VehicleAttitude
from rclpy.clock import Clock
import numpy as np


class UAV:
    """
    Skeleton class for UAV control and interfacing with PX4 via ROS 2.
    """

    def __init__(self, node: Node):
        self.node = node
        
        # Subscribers
        self._initialize_publishers_and_subscribers()

        # Timers
        self._initialize_timers()

    def _initialize_publishers_and_subscribers(self):
        """
        Initialize ROS 2 publishers and subscribers.
        """
        pass

    def _initialize_timers(self):
        """
        Initialize ROS 2 timers.
        """
        pass
    
    # Callbacks
    def _first_callback(self, msg: OffboardControlMode):
        """
        Example callback for subscriber to OffboardControlMode messages.
        """
        pass
    
    # Public methods
    def arm(self):
        """
        Send an arm command to the UAV.
        """
        pass

    def disarm(self):
        """
        Send a disarm command to the UAV.
        """
        pass

    def takeoff(self, altitude: float = 5.0):
        """
        Command the UAV to take off to the specified altitude.
        """
        pass

    def land(self):
        """
        Command the UAV to land.
        """
        pass

    def set_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float):
        """
        Set velocity commands for offboard control.
        """
        pass

    # Internal helper methods
    def _send_vehicle_command(self, command: int, params: dict = {}):
        """
        Publish a VehicleCommand message.
        """
        pass