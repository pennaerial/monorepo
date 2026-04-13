"""
Reusable mixins for inter-payload state synchronization.

Two code paths depending on whether the sync can run in-process:

- **Direct (in-process):** ``emits_hotspot=True`` AND ``sync_is_local``.
  Creates ROS pub/sub directly on ``self.node``.

- **Subprocess (external node):** ``emits_hotspot=False`` OR sync domain
  differs from the main mission domain.  Launches a non-blocking bash
  script that optionally pings ``TARGET_IP``, exports the correct
  ``ROS_DOMAIN_ID``, sources ROS, and starts ``minimal_sync_node.py``.
  Communication between mode and external node is via temp files.
"""

from __future__ import annotations

import os
import signal
import subprocess
from typing import Optional

from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String

SYNC_TOPIC = "/payload_sync"
PUB_STATE_FILE = "/tmp/payload_sync_state"
SUB_RECV_FILE = "/tmp/payload_sync_recv"


def _minimal_sync_node_path() -> str:
    share = get_package_share_directory("uav")
    return os.path.join(share, "scripts", "minimal_sync_node.py")


def _needs_subprocess(node) -> bool:
    return not node.emits_hotspot or not node.sync_is_local


def _build_bash_script(node, mode: str) -> str:
    """Build the bash script that optionally pings, sets domain, and starts the node."""
    lines = ["#!/bin/bash", "set -e"]

    if not node.emits_hotspot:
        lines += [
            f'TARGET_IP="{node.target_ip}"',
            "while ! ping -c 1 -W 1 $TARGET_IP &> /dev/null; do sleep 1; done",
        ]

    lines.append(f"export ROS_DOMAIN_ID={node.sync_domain_id}")
    lines.append("source /opt/ros/humble/setup.bash")
    lines.append(f"exec python3 {_minimal_sync_node_path()} {mode}")

    return "\n".join(lines)


class PayloadSyncPublisherMixin:
    """Mixin that adds ``/payload_sync`` publishing to a Mode."""

    _sync_publisher: Optional[object] = None
    _sync_pub_process: Optional[subprocess.Popen] = None

    def _sync_pub_enter(self) -> None:
        if _needs_subprocess(self.node):
            try:
                os.remove(PUB_STATE_FILE)
            except FileNotFoundError:
                pass
            script = _build_bash_script(self.node, "pub")
            self._sync_pub_process = subprocess.Popen(
                ["bash", "-c", script],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self.node.get_logger().info(
                f"[sync] publisher subprocess started (pid={self._sync_pub_process.pid})"
            )
        else:
            self._sync_publisher = self.node.create_publisher(String, SYNC_TOPIC, 10)
            self.node.get_logger().info("[sync] direct publisher created")

    def _sync_publish(self, substate: str) -> None:
        if _needs_subprocess(self.node):
            try:
                with open(PUB_STATE_FILE, "w") as f:
                    f.write(substate)
            except OSError:
                pass
        else:
            if self._sync_publisher is not None:
                self._sync_publisher.publish(String(data=substate))

    def _sync_pub_exit(self) -> None:
        if self._sync_pub_process is not None:
            try:
                os.kill(self._sync_pub_process.pid, signal.SIGTERM)
                self._sync_pub_process.wait(timeout=3)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                self._sync_pub_process.kill()
            self._sync_pub_process = None
            self.node.get_logger().info("[sync] publisher subprocess terminated")
            try:
                os.remove(PUB_STATE_FILE)
            except FileNotFoundError:
                pass
        if self._sync_publisher is not None:
            self.node.destroy_publisher(self._sync_publisher)
            self._sync_publisher = None
            self.node.get_logger().info("[sync] direct publisher destroyed")


class PayloadSyncSubscriberMixin:
    """Mixin that adds ``/payload_sync`` subscribing to a Mode."""

    _sync_subscription: Optional[object] = None
    _sync_sub_process: Optional[subprocess.Popen] = None
    _sync_target_reached: bool = False
    _sync_wait_for_state: str = ""
    _sync_latest_state: str = ""

    def _sync_sub_enter(self, wait_for_state: str) -> None:
        self._sync_target_reached = False
        self._sync_wait_for_state = wait_for_state
        self._sync_latest_state = ""

        if _needs_subprocess(self.node):
            try:
                os.remove(SUB_RECV_FILE)
            except FileNotFoundError:
                pass
            script = _build_bash_script(self.node, "sub")
            self._sync_sub_process = subprocess.Popen(
                ["bash", "-c", script],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self.node.get_logger().info(
                f"[sync] subscriber subprocess started (pid={self._sync_sub_process.pid})"
            )
        else:
            self._sync_subscription = self.node.create_subscription(
                String, SYNC_TOPIC, self._sync_on_msg, 10
            )
            self.node.get_logger().info("[sync] direct subscriber created")

    def _sync_on_msg(self, msg: String) -> None:
        self._sync_latest_state = msg.data
        if msg.data == self._sync_wait_for_state:
            self._sync_target_reached = True
            self.node.get_logger().info(
                f"[sync] target state '{self._sync_wait_for_state}' received"
            )

    def _sync_check_target(self) -> bool:
        if self._sync_target_reached:
            return True

        if _needs_subprocess(self.node):
            try:
                with open(SUB_RECV_FILE, "r") as f:
                    state = f.read().strip()
            except FileNotFoundError:
                return False
            if state:
                self._sync_latest_state = state
                if state == self._sync_wait_for_state:
                    self._sync_target_reached = True
                    self.node.get_logger().info(
                        f"[sync] target state '{self._sync_wait_for_state}' received (via file)"
                    )

        return self._sync_target_reached

    def _sync_sub_exit(self) -> None:
        if self._sync_sub_process is not None:
            try:
                os.kill(self._sync_sub_process.pid, signal.SIGTERM)
                self._sync_sub_process.wait(timeout=3)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                self._sync_sub_process.kill()
            self._sync_sub_process = None
            self.node.get_logger().info("[sync] subscriber subprocess terminated")
            try:
                os.remove(SUB_RECV_FILE)
            except FileNotFoundError:
                pass
        if self._sync_subscription is not None:
            self.node.destroy_subscription(self._sync_subscription)
            self._sync_subscription = None
            self.node.get_logger().info("[sync] direct subscriber destroyed")
