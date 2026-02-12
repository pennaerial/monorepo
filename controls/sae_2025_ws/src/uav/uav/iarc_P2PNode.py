#!/usr/bin/env python3

from __future__ import annotations

from typing import Optional, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from uav_interfaces.msg import DroneState, MineDetection, ConnectionStatus


class P2PPNode(Node):
	"""Manages P2P communication link between two drones.
	
	This node:
	- Subscribes to peer's DroneState (position, mode, battery, status)
	- Subscribes to peer's MineDetection events  
	- Tracks connection health (connected/degraded/lost)
	- Does NOT aggregate full swarm - only manages this one pairwise link
	
	Each drone runs multiple instances of this node (one per peer).
	Connection loss does NOT mean peer drone failed - just that this link is down.
	"""

	def __init__(self) -> None:
		super().__init__('p2p_pair_link')

		self.declare_parameter('vehicle_id', 0)
		self.declare_parameter('peer_id', 1)
		self.declare_parameter('topic_prefix', '/p2p')
		self.declare_parameter('connection_timeout_sec', 3.0)
		self.declare_parameter('log_hz', 1.0)

		self.vehicle_id = int(self.get_parameter('vehicle_id').value)
		self.peer_id = int(self.get_parameter('peer_id').value)
		self.topic_prefix = str(self.get_parameter('topic_prefix').value).rstrip('/')
		self.connection_timeout_ns = int(float(self.get_parameter('connection_timeout_sec').value) * 1e9)
		self.log_hz = float(self.get_parameter('log_hz').value)

		if self.vehicle_id == self.peer_id:
			raise ValueError(f"vehicle_id and peer_id cannot be the same ({self.vehicle_id})")

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

		# Track peer state
		self._peer_state: Optional[DroneState] = None
		self._last_peer_update_ns: int = 0
		self._connection_status: str = "disconnected"  # "connected", "degraded", "lost"
		self._peer_mines: List[MineDetection] = []

		# Subscribe to peer's state
		peer_state_topic = f"{self.topic_prefix}/vehicle_{self.peer_id}/state"
		self._peer_state_sub = self.create_subscription(
			DroneState,
			peer_state_topic,
			self._on_peer_state,
			qos_state,
		)

		# Subscribe to peer's mine detections
		peer_mine_topic = f"{self.topic_prefix}/vehicle_{self.peer_id}/mine_detection"
		self._peer_mine_sub = self.create_subscription(
			MineDetection,
			peer_mine_topic,
			self._on_peer_mine_detection,
			qos_critical,
		)

		# Publish connection status for other nodes (mode manager, etc.)
		connection_status_topic = f"{self.topic_prefix}/vehicle_{self.vehicle_id}/connection_{self.peer_id}"
		self._connection_status_pub = self.create_publisher(
			ConnectionStatus,
			connection_status_topic,
			qos_state,
		)

		# Timer for connection health monitoring
		self.create_timer(1.0, self._check_connection_health)

		# Timer for logging
		if self.log_hz > 0.0:
			self._log_timer = self.create_timer(1.0 / self.log_hz, self._log_status)
		else:
			self._log_timer = None

		self.get_logger().info(
			f"P2P Pair Link started: vehicle_{self.vehicle_id} ←→ vehicle_{self.peer_id}"
		)

	def _on_peer_state(self, msg: DroneState) -> None:
		"""Callback for receiving peer's state update."""
		self._peer_state = msg
		self._last_peer_update_ns = self.get_clock().now().nanoseconds
		
		# Update connection status to connected when we receive data
		if self._connection_status != "connected":
			self._connection_status = "connected"
			self.get_logger().info(f"Connection to vehicle_{self.peer_id} established")

	def _on_peer_mine_detection(self, msg: MineDetection) -> None:
		"""Callback for receiving mine detection from peer."""
		self._peer_mines.append(msg)
		# Keep last 50 detections
		if len(self._peer_mines) > 50:
			self._peer_mines.pop(0)
		
		self.get_logger().info(
			f"Peer vehicle_{self.peer_id} detected mine at "
			f"({msg.detection_pose.pose.position.x:.2f}, "
			f"{msg.detection_pose.pose.position.y:.2f}) "
			f"confidence: {msg.confidence:.2f}"
		)

	def _check_connection_health(self) -> None:
		"""Monitor connection health based on last message time."""
		now_ns = self.get_clock().now().nanoseconds
		
		if self._last_peer_update_ns == 0:
			# Never received a message
			self._connection_status = "disconnected"
			age_sec = 0.0
		else:
			age_ns = now_ns - self._last_peer_update_ns
			age_sec = age_ns / 1e9
			
			if age_ns < self.connection_timeout_ns:
				self._connection_status = "connected"
			elif age_ns < (self.connection_timeout_ns * 2):
				if self._connection_status != "degraded":
					self._connection_status = "degraded"
					self.get_logger().warn(
						f"Connection to vehicle_{self.peer_id} degraded (no update for {age_sec:.1f}s)"
					)
			else:
				if self._connection_status != "lost":
					self._connection_status = "lost"
					self.get_logger().error(
						f"Connection to vehicle_{self.peer_id} lost (no update for {age_sec:.1f}s)"
					)
		
		# Publish connection status for other nodes to consume
		status_msg = ConnectionStatus()
		status_msg.my_vehicle_id = self.vehicle_id
		status_msg.peer_id = self.peer_id
		status_msg.link_status = self._connection_status
		status_msg.last_peer_update = self.get_clock().now().to_msg()
		status_msg.age_seconds = age_sec
		
		self._connection_status_pub.publish(status_msg)

	def _log_status(self) -> None:
		"""Log current P2P link status."""
		if self._peer_state is None:
			self.get_logger().info(
				f"Link: v{self.vehicle_id}←→v{self.peer_id} | Status: {self._connection_status} | No data"
			)
			return
		
		now_ns = self.get_clock().now().nanoseconds
		age_s = (now_ns - self._last_peer_update_ns) / 1e9
		
		pos = self._peer_state.pose.pose.position
		self.get_logger().info(
			f"Link: v{self.vehicle_id}←→v{self.peer_id} | "
			f"Status: {self._connection_status} | "
			f"Peer at ({pos.x:.1f}, {pos.y:.1f}, {pos.z:.1f}) | "
			f"Mode: {self._peer_state.flight_mode} | "
			f"Battery: {self._peer_state.battery_level:.1f}% | "
			f"Age: {age_s:.1f}s | "
			f"Mines: {len(self._peer_mines)}"
		)

	def get_peer_state(self) -> Optional[DroneState]:
		"""Get the latest peer state (for other nodes to query)."""
		return self._peer_state

	def get_peer_mines(self) -> List[MineDetection]:
		"""Get all mine detections from peer."""
		return self._peer_mines.copy()

	def is_connected(self) -> bool:
		"""Check if connected to peer."""
		return self._connection_status == "connected"


def main(args=None) -> None:
	rclpy.init(args=args)
	node = None
	try:
		node = P2PPairNode()
		rclpy.spin(node)
	finally:
		if node is not None:
			node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
