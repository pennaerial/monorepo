#!/usr/bin/env python3
"""
Minimal ROS 2 node for inter-payload state synchronization.

Usage:
    python3 minimal_sync_node.py pub          # publisher mode
    python3 minimal_sync_node.py sub          # subscriber mode

Publisher mode:
    Polls /tmp/payload_sync_state for the current substate string and
    publishes it on /payload_sync at ~10 Hz.

Subscriber mode:
    Subscribes to /payload_sync and writes every received message to
    /tmp/payload_sync_recv (overwrites on each message).
"""

from __future__ import annotations

import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TOPIC = "/payload_sync"
PUB_STATE_FILE = "/tmp/payload_sync_state"
SUB_RECV_FILE = "/tmp/payload_sync_recv"
PUBLISH_HZ = 10.0


class SyncPublisher(Node):
    def __init__(self) -> None:
        super().__init__("payload_sync_pub")
        self._pub = self.create_publisher(String, TOPIC, 10)
        self._timer = self.create_timer(1.0 / PUBLISH_HZ, self._tick)
        self._last_state = ""

    def _tick(self) -> None:
        try:
            with open(PUB_STATE_FILE, "r") as f:
                state = f.read().strip()
        except FileNotFoundError:
            return
        if not state:
            return
        self._last_state = state
        msg = String(data=state)
        self._pub.publish(msg)


class SyncSubscriber(Node):
    def __init__(self) -> None:
        super().__init__("payload_sync_sub")
        self.create_subscription(String, TOPIC, self._on_msg, 10)

    def _on_msg(self, msg: String) -> None:
        with open(SUB_RECV_FILE, "w") as f:
            f.write(msg.data)


def main() -> None:
    if len(sys.argv) < 2 or sys.argv[1] not in ("pub", "sub"):
        print(f"Usage: {sys.argv[0]} pub|sub", file=sys.stderr)
        sys.exit(1)

    mode = sys.argv[1]
    rclpy.init(args=None)

    node = SyncPublisher() if mode == "pub" else SyncSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
