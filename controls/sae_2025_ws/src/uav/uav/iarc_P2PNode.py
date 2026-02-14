#!/usr/bin/env python3

from __future__ import annotations

from typing import Optional, List, Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus, BatteryStatus
from std_msgs.msg import String
from uav_interfaces.msg import DroneState, MineDetection, ConnectionStatus


class P2PNode(Node):
	"""P2P communication node for each drone in a distributed system.
	
	This node:
	- Publishes own DroneState (position, mode, battery, status) from PX4
	- Subscribes to all peer DroneState messages
	- Subscribes to all peer MineDetection events
	- Tracks connection health to each peer (connected/degraded/lost)
	
	Each drone runs ONE instance of this node.
	"""

	def __init__(self) -> None:
		super().__init__('p2p_node')

		self.declare_parameter('vehicle_id', 0)
		self.declare_parameter('peer_ids', [1])
		self.declare_parameter('topic_prefix', '/p2p')
		self.declare_parameter('connection_timeout_sec', 3.0)
		self.declare_parameter('log_hz', 1.0)
		self.declare_parameter('publish_hz', 5.0)
		self.declare_parameter('frame_id', 'map')

		self.vehicle_id = int(self.get_parameter('vehicle_id').value)
		self.peer_ids = list(self.get_parameter('peer_ids').value)
		self.topic_prefix = str(self.get_parameter('topic_prefix').value).rstrip('/')
		self.connection_timeout_ns = int(float(self.get_parameter('connection_timeout_sec').value) * 1e9)
		self.log_hz = float(self.get_parameter('log_hz').value)
		self.publish_hz = float(self.get_parameter('publish_hz').value)
		self.frame_id = str(self.get_parameter('frame_id').value)

		# Validate peer_ids
		if self.vehicle_id in self.peer_ids:
			raise ValueError(f"vehicle_id {self.vehicle_id} cannot be in peer_ids list")
		
		# Auto-derive PX4 topics based on vehicle_id
		px4_prefix = '' if self.vehicle_id == 0 else f"/px4_{self.vehicle_id}"
		local_position_topic = f"{px4_prefix}/fmu/out/vehicle_local_position"
		vehicle_status_topic = f"{px4_prefix}/fmu/out/vehicle_status"
		battery_topic = f"{px4_prefix}/fmu/out/battery_status"

		# QoS: BEST_EFFORT for frequent updates, RELIABLE for critical messages
		qos_state = QoSProfile(
			reliability=QoSReliabilityPolicy.BEST_EFFORT,
			durability=QoSDurabilityPolicy.VOLATILE,
			history=QoSHistoryPolicy.KEEP_LAST,
			depth=10,
		)
		qos_critical = QoSProfile(
			reliability=QoSReliabilityPolicy.RELIABLE,
			durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
			history=QoSHistoryPolicy.KEEP_LAST,
			depth=10,
		)

		# Track own state for publishing
		self._latest_local_position: Optional[VehicleLocalPosition] = None
		self._flight_mode: int = 0
		self._battery_level: float = 0.0
		self._status: str = "unknown"
		self._is_armed: bool = False
		self._autonomous_mode: str = "unknown"

		# Track peer states: {peer_id: DroneState}
		self._peer_states: Dict[int, Optional[DroneState]] = {pid: None for pid in self.peer_ids}
		self._last_peer_updates: Dict[int, int] = {pid: 0 for pid in self.peer_ids}
		self._connection_statuses: Dict[int, str] = {pid: "disconnected" for pid in self.peer_ids}
		self._peer_mines: Dict[int, List[MineDetection]] = {pid: [] for pid in self.peer_ids}

		# Subscribe to own PX4 data for publishing local state
		self.create_subscription(
			VehicleLocalPosition,
			local_position_topic,
			self._on_local_position,
			qos_state,
		)
		self.create_subscription(
			VehicleStatus,
			vehicle_status_topic,
			self._on_vehicle_status,
			qos_state,
		)
		self.create_subscription(
			BatteryStatus,
			battery_topic,
			self._on_battery_status,
			qos_state,
		)

		# Subscribe to autonomous mode from ModeManager
		mode_topic = f"/vehicle_{self.vehicle_id}/current_mode"
		self.create_subscription(
			String,
			mode_topic,
			self._on_autonomous_mode,
			10,
		)

		# Publish own state
		state_topic = f"{self.topic_prefix}/vehicle_{self.vehicle_id}/state"
		self._state_pub = self.create_publisher(DroneState, state_topic, qos_state)
		self.create_timer(1.0 / self.publish_hz, self._publish_state)

		# Initialize dict for connection status publishers
		self._connection_pubs: Dict[int, Any] = {}

		# Subscribe to each peer's state and mine detections
		for peer_id in self.peer_ids:
			# Subscribe to peer state
			peer_state_topic = f"{self.topic_prefix}/vehicle_{peer_id}/state"
			self.create_subscription(
				DroneState,
				peer_state_topic,
				lambda msg, pid=peer_id: self._on_peer_state(msg, pid),
				qos_state,
			)

			# Subscribe to peer mine detections
			peer_mine_topic = f"{self.topic_prefix}/vehicle_{peer_id}/mine_detection"
			self.create_subscription(
				MineDetection,
				peer_mine_topic,
				lambda msg, pid=peer_id: self._on_peer_mine_detection(msg, pid),
				qos_critical,
			)

			# Publish connection status for observability
			connection_status_topic = f"{self.topic_prefix}/vehicle_{self.vehicle_id}/connection_{peer_id}"
			conn_pub = self.create_publisher(ConnectionStatus, connection_status_topic, qos_state)
			self._connection_pubs[peer_id] = conn_pub

		# Timer for connection health monitoring
		self.create_timer(1.0, self._check_connection_health)

		# Timer for logging
		if self.log_hz > 0.0:
			self._log_timer = self.create_timer(1.0 / self.log_hz, self._log_status)
		else:
			self._log_timer = None

		self.get_logger().info(
			f"P2P Node started: vehicle_{self.vehicle_id} monitoring peers {self.peer_ids}"
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
		"""Publish current drone state to P2P network."""
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

	def _on_peer_state(self, msg: DroneState, peer_id: int) -> None:
		"""Callback for receiving peer's state update."""
		self._peer_states[peer_id] = msg
		self._last_peer_updates[peer_id] = self.get_clock().now().nanoseconds
		
		# Update connection status to connected when we receive data
		if self._connection_statuses[peer_id] != "connected":
			self._connection_statuses[peer_id] = "connected"
			self.get_logger().info(f"Connection to vehicle_{peer_id} established")

	def _on_peer_mine_detection(self, msg: MineDetection, peer_id: int) -> None:
		"""Callback for receiving mine detection from peer."""
		self._peer_mines[peer_id].append(msg)
		# Keep last 50 detections per peer
		if len(self._peer_mines[peer_id]) > 50:
			self._peer_mines[peer_id].pop(0)
		
		self.get_logger().info(
			f"Peer vehicle_{peer_id} detected mine at "
			f"({msg.detection_pose.pose.position.x:.2f}, "
			f"{msg.detection_pose.pose.position.y:.2f}) "
			f"confidence: {msg.confidence:.2f}"
		)

	def _check_connection_health(self) -> None:
		"""Monitor connection health for all peers.
		
		Publishes ConnectionStatus for observability/monitoring purposes only.
		Connection loss does NOT trigger mission changes in this distributed system.
		"""
		now_ns = self.get_clock().now().nanoseconds
		
		for peer_id in self.peer_ids:
			if self._last_peer_updates[peer_id] == 0:
				# Never received a message
				self._connection_statuses[peer_id] = "disconnected"
				age_sec = 0.0
			else:
				age_ns = now_ns - self._last_peer_updates[peer_id]
				age_sec = age_ns / 1e9
				
				if age_ns < self.connection_timeout_ns:
					self._connection_statuses[peer_id] = "connected"
				elif age_ns < (self.connection_timeout_ns * 2):
					if self._connection_statuses[peer_id] != "degraded":
						self._connection_statuses[peer_id] = "degraded"
						self.get_logger().warn(
							f"Connection to vehicle_{peer_id} degraded (no update for {age_sec:.1f}s)"
						)
				else:
					if self._connection_statuses[peer_id] != "lost":
						self._connection_statuses[peer_id] = "lost"
						self.get_logger().error(
							f"Connection to vehicle_{peer_id} lost (no update for {age_sec:.1f}s)"
						)
			
			# Publish connection status for this peer
			status_msg = ConnectionStatus()
			status_msg.my_vehicle_id = self.vehicle_id
			status_msg.peer_id = peer_id
			status_msg.link_status = self._connection_statuses[peer_id]
			status_msg.last_peer_update = self.get_clock().now().to_msg()
			status_msg.age_seconds = age_sec
			
			self._connection_pubs[peer_id].publish(status_msg)

	def _log_status(self) -> None:
		"""Log current P2P network status."""
		self.get_logger().info(
			f"--- P2P Status (vehicle_{self.vehicle_id}) ---"
		)
		
		# Log own state
		if self._latest_local_position:
			pos = self._latest_local_position
			self.get_logger().info(
				f"Own: pos=({pos.x:.1f}, {pos.y:.1f}, {pos.z:.1f}) "
				f"mode={self._autonomous_mode} battery={self._battery_level:.1f}% "
				f"status={self._status}"
			)
		
		# Log each peer
		for peer_id in self.peer_ids:
			peer_state = self._peer_states[peer_id]
			conn_status = self._connection_statuses[peer_id]
			mine_count = len(self._peer_mines[peer_id])
			
			if peer_state is None:
				self.get_logger().info(
					f"Peer {peer_id}: {conn_status} | No data received"
				)
			else:
				pos = peer_state.pose.pose.position
				now_ns = self.get_clock().now().nanoseconds
				age_s = (now_ns - self._last_peer_updates[peer_id]) / 1e9
				
				self.get_logger().info(
					f"Peer {peer_id}: {conn_status} | "
					f"pos=({pos.x:.1f}, {pos.y:.1f}, {pos.z:.1f}) "
					f"mode={peer_state.autonomous_mode} "
					f"battery={peer_state.battery_level:.1f}% "
					f"age={age_s:.1f}s mines={mine_count}"
				)

	def get_peer_state(self, peer_id: int) -> Optional[DroneState]:
		"""Get the latest state for a specific peer."""
		return self._peer_states.get(peer_id)

	def get_all_peer_states(self) -> Dict[int, Optional[DroneState]]:
		"""Get all peer states."""
		return self._peer_states.copy()

	def get_peer_mines(self, peer_id: int) -> List[MineDetection]:
		"""Get all mine detections from a specific peer."""
		return self._peer_mines.get(peer_id, []).copy()

	def get_all_mines(self) -> List[MineDetection]:
		"""Get all mine detections from all peers."""
		all_mines = []
		for mines in self._peer_mines.values():
			all_mines.extend(mines)
		return all_mines

	def is_connected(self, peer_id: int) -> bool:
		"""Check if connected to specific peer."""
		return self._connection_statuses.get(peer_id) == "connected"


def main(args=None) -> None:
	rclpy.init(args=args)
	node = None
	try:
		node = P2PNode()
		rclpy.spin(node)
	finally:
		if node is not None:
			node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
