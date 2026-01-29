
#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleLocalPosition


def _parse_peer_ids(value) -> List[int]:
	if value is None:
		return []
	if isinstance(value, list):
		out: List[int] = []
		for item in value:
			if isinstance(item, (int, float)):
				out.append(int(item))
			elif isinstance(item, str) and item.strip():
				out.append(int(item.strip()))
		return sorted(set(out))
	if isinstance(value, str):
		s = value.strip()
		if not s:
			return []
		return sorted({int(x.strip()) for x in s.split(',') if x.strip()})
	if isinstance(value, (int, float)):
		return [int(value)]
	return []


@dataclass
class PeerState:
	pose: PoseStamped
	last_update_ns: int


class P2PCommTest(Node):
	"""Publish this vehicle's local position and subscribe to peer vehicles' positions.

	Intended for quick multi-vehicle comms testing.

	- Subscribes to PX4 local position: `source_topic` (default `/fmu/out/vehicle_local_position`)
	- Publishes PoseStamped to: `{topic_prefix}/vehicle_<id>/pose`
	- Subscribes to peers: same topic pattern for each id in `peer_ids`

	Notes:
	- `VehicleLocalPosition` is in PX4 local NED frame (z is down). This test publishes
	  the values directly into PoseStamped position (x,y,z).
	"""

	def __init__(self) -> None:
		super().__init__('p2p_comm_test')

		self.declare_parameter('vehicle_id', 0)
		self.declare_parameter('peer_ids', 0)
		self.declare_parameter('topic_prefix', '/swarm')
		# If empty or 'auto', derive from vehicle_id:
		# - 0: /fmu/out/vehicle_local_position
		# - n: /px4_n/fmu/out/vehicle_local_position
		self.declare_parameter('source_topic', 'auto')
		self.declare_parameter('publish_hz', 5.0)
		self.declare_parameter('frame_id', 'map')
		self.declare_parameter('log_hz', 1.0)

		self.vehicle_id = int(self.get_parameter('vehicle_id').value)
		self.topic_prefix = str(self.get_parameter('topic_prefix').value).rstrip('/')
		configured_source_topic = str(self.get_parameter('source_topic').value).strip()
		if configured_source_topic == '' or configured_source_topic.lower() == 'auto':
			prefix = '' if self.vehicle_id == 0 else f"/px4_{self.vehicle_id}"
			self.source_topic = f"{prefix}/fmu/out/vehicle_local_position"
		else:
			self.source_topic = configured_source_topic
		self.publish_hz = float(self.get_parameter('publish_hz').value)
		self.frame_id = str(self.get_parameter('frame_id').value)
		self.log_hz = float(self.get_parameter('log_hz').value)

		peer_param = self.get_parameter('peer_ids').value
		self.peer_ids = [pid for pid in _parse_peer_ids(peer_param) if pid != self.vehicle_id]

		if self.publish_hz <= 0.0:
			raise ValueError(f"publish_hz must be > 0, got {self.publish_hz}")

		qos = QoSProfile(
			reliability=QoSReliabilityPolicy.BEST_EFFORT,
			durability=QoSDurabilityPolicy.VOLATILE,
			history=QoSHistoryPolicy.KEEP_LAST,
			depth=10,
		)

		self._latest_local: Optional[VehicleLocalPosition] = None
		self._peers: Dict[int, PeerState] = {}

		self._pose_pub = self.create_publisher(PoseStamped, self._pose_topic(self.vehicle_id), qos)
		self._local_sub = self.create_subscription(
			VehicleLocalPosition,
			self.source_topic,
			self._on_local_position,
			qos,
		)

		self._peer_subs = []
		for peer_id in self.peer_ids:
			topic = self._pose_topic(peer_id)
			self._peer_subs.append(
				self.create_subscription(
					PoseStamped,
					topic,
					lambda msg, pid=peer_id: self._on_peer_pose(pid, msg),
					qos,
				)
			)

		self._pub_timer = self.create_timer(1.0 / self.publish_hz, self._publish_pose)
		if self.log_hz > 0.0:
			self._log_timer = self.create_timer(1.0 / self.log_hz, self._log_status)
		else:
			self._log_timer = None

		self.get_logger().info(
			f"p2p_comm_test started (vehicle_id={self.vehicle_id}, peers={self.peer_ids}). "
			f"source_topic='{self.source_topic}', pub='{self._pose_topic(self.vehicle_id)}'"
		)

	def _pose_topic(self, vehicle_id: int) -> str:
		return f"{self.topic_prefix}/vehicle_{vehicle_id}/pose"

	def _on_local_position(self, msg: VehicleLocalPosition) -> None:
		self._latest_local = msg

	def _publish_pose(self) -> None:
		if self._latest_local is None:
			return

		pose = PoseStamped()
		pose.header.stamp = self.get_clock().now().to_msg()
		pose.header.frame_id = self.frame_id

		# PX4 local position is NED. We publish it as-is for test purposes.
		pose.pose.position.x = float(self._latest_local.x)
		pose.pose.position.y = float(self._latest_local.y)
		pose.pose.position.z = float(self._latest_local.z)

		# No attitude here; identity quaternion.
		pose.pose.orientation.w = 1.0

		self._pose_pub.publish(pose)

	def _on_peer_pose(self, peer_id: int, msg: PoseStamped) -> None:
		self._peers[peer_id] = PeerState(pose=msg, last_update_ns=self.get_clock().now().nanoseconds)

	def _log_status(self) -> None:
		now_ns = self.get_clock().now().nanoseconds
		if not self.peer_ids:
			return

		parts = []
		for peer_id in self.peer_ids:
			state = self._peers.get(peer_id)
			if state is None:
				parts.append(f"{peer_id}:no-msg")
				continue
			age_s = (now_ns - state.last_update_ns) / 1e9
			x = state.pose.pose.position.x
			y = state.pose.pose.position.y
			z = state.pose.pose.position.z
			parts.append(f"{peer_id}:(x={x:.2f},y={y:.2f},z={z:.2f},age={age_s:.1f}s)")

		self.get_logger().info("peers " + " ".join(parts))


def main(args=None) -> None:
	rclpy.init(args=args)
	node = None
	try:
		node = P2PCommTest()
		rclpy.spin(node)
	finally:
		if node is not None:
			node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()

