#!/usr/bin/env python3

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus, BatteryStatus
from std_msgs.msg import String
from uav_interfaces.msg import DroneState


class LocalStatePublisher(Node):
	"""Publishes this drone's state (position, flight mode, battery, status) to swarm.
	
	Subscribes to:
	- PX4 local position
	- Flight mode
	- Battery info
	
	Publishes to:
	- /swarm/vehicle_{id}/state (DroneState message)
	"""

	def __init__(self) -> None:
		super().__init__('local_state_publisher')

		self.declare_parameter('vehicle_id', 0)
		self.declare_parameter('topic_prefix', '/p2p')
		self.declare_parameter('publish_hz', 5.0)
		self.declare_parameter('frame_id', 'map')

		self.vehicle_id = int(self.get_parameter('vehicle_id').value)
		self.topic_prefix = str(self.get_parameter('topic_prefix').value).rstrip('/')
		self.publish_hz = float(self.get_parameter('publish_hz').value)
		self.frame_id = str(self.get_parameter('frame_id').value)

		# Auto-derive PX4 topics based on vehicle_id
		prefix = '' if self.vehicle_id == 0 else f"/px4_{self.vehicle_id}"
		local_position_topic = f"{prefix}/fmu/out/vehicle_local_position"
		flight_mode_topic = f"{prefix}/fmu/out/vehicle_status"
		battery_topic = f"{prefix}/fmu/out/battery_status"

		# QoS for local sensor data (BEST_EFFORT)
		qos_sensor = QoSProfile(
			reliability=QoSReliabilityPolicy.BEST_EFFORT,
			durability=QoSDurabilityPolicy.VOLATILE,
			history=QoSHistoryPolicy.KEEP_LAST,
			depth=10,
		)

		# Latest sensor readings (fallbacks)
		self._latest_local_position: VehicleLocalPosition | None = None
		self._flight_mode: int = 0
		self._battery_level: float = 0.0
		self._status: str = "unknown"
		self._is_armed: bool = False
		self._autonomous_mode: str = "unknown"

		# Subscribe to local position
		self.create_subscription(
			VehicleLocalPosition,
			local_position_topic,
			self._on_local_position,
			qos_sensor,
		)

		# Subscribe to vehicle status (flight mode, arming state)
		self.create_subscription(
			VehicleStatus,
			flight_mode_topic,
			self._on_vehicle_status,
			qos_sensor,
		)

		# Subscribe to battery status
		self.create_subscription(
			BatteryStatus,
			battery_topic,
			self._on_battery_status,
			qos_sensor,
		)

		# Subscribe to autonomous mode from ModeManager
		mode_topic = f"/vehicle_{self.vehicle_id}/current_mode"
		self.create_subscription(
			String,
			mode_topic,
			self._on_autonomous_mode,
			10,
		)

		# Publish DroneState
		qos_publish = QoSProfile(
			reliability=QoSReliabilityPolicy.BEST_EFFORT,
			durability=QoSDurabilityPolicy.VOLATILE,
			history=QoSHistoryPolicy.KEEP_LAST,
			depth=10,
		)
		
		state_topic = f"{self.topic_prefix}/vehicle_{self.vehicle_id}/state"
		self._state_pub = self.create_publisher(DroneState, state_topic, qos_publish)
		
		# Publish at desired rate
		self.create_timer(1.0 / self.publish_hz, self._publish_state)

		self.get_logger().info(
			f"LocalStatePublisher started (vehicle_id={self.vehicle_id}, "
			f"pub_hz={self.publish_hz})"
		)

	def _on_local_position(self, msg: VehicleLocalPosition) -> None:
		"""Update position from PX4."""
		self._latest_local_position = msg

	def _on_vehicle_status(self, msg: VehicleStatus) -> None:
		"""Update flight mode and arming state from PX4."""
		self._flight_mode = msg.nav_state
		self._is_armed = (msg.arming_state == 2)  # 2 = ARMED
		
		# Derive status from various state fields
		if msg.failure_detector_status & 0x01:  # Failure detected
			self._status = "error"
		elif msg.nav_state == 18:  # AUTO_RTL
			self._status = "returning"
		elif not self._is_armed:
			self._status = "disarmed"
		elif self._is_armed:
			self._status = "armed"
		else:
			self._status = "unknown"

	def _on_battery_status(self, msg: BatteryStatus) -> None:
		"""Update battery level from PX4."""
		# PX4 reports remaining as 0.0-1.0, convert to percentage
		self._battery_level = msg.remaining * 100.0

	def _on_autonomous_mode(self, msg: String) -> None:
		"""Update autonomous mode from ModeManager."""
		self._autonomous_mode = msg.data

	def _publish_state(self) -> None:
		"""Publish current drone state."""
		if self._latest_local_position is None:
			return

		# Build geometry_msgs/PoseStamped
		pose = PoseStamped()
		pose.header.stamp = self.get_clock().now().to_msg()
		pose.header.frame_id = self.frame_id
		pose.pose.position.x = float(self._latest_local_position.x)
		pose.pose.position.y = float(self._latest_local_position.y)
		pose.pose.position.z = float(self._latest_local_position.z)
		pose.pose.orientation.w = 1.0  # Identity quaternion

		# Build DroneState
		state = DroneState()
		state.vehicle_id = self.vehicle_id
		state.pose = pose
		state.flight_mode = self._flight_mode
		state.battery_level = self._battery_level
		state.status = self._status
		state.is_armed = self._is_armed
		state.autonomous_mode = self._autonomous_mode

		self._state_pub.publish(state)


def main(args=None) -> None:
	rclpy.init(args=args)
	node = None
	try:
		node = LocalStatePublisher()
		rclpy.spin(node)
	finally:
		if node is not None:
			node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
