#!/usr/bin/env python3
"""
Live plotter for payload motor state RPM traces.

Usage:
  python3 src/payload/test/plot_motor_state.py --topic /payload_0/motor_state
"""

import argparse
import threading
import time
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import rclpy
from rclpy.node import Node

from payload_interfaces.msg import MotorState


class MotorStatePlotter(Node):
    def __init__(self, topic: str, window_sec: float) -> None:
        super().__init__("motor_state_plotter")
        self._window_sec = window_sec
        self._start_time = time.time()
        self._lock = threading.Lock()

        self._t = deque()
        self._left_set = deque()
        self._left_meas = deque()
        self._right_set = deque()
        self._right_meas = deque()

        self._sub = self.create_subscription(
            MotorState,
            topic,
            self._callback,
            50,
        )
        self.get_logger().info(f"Subscribing to {topic}")

    def _trim(self, now: float) -> None:
        min_t = now - self._window_sec
        while self._t and self._t[0] < min_t:
            self._t.popleft()
            self._left_set.popleft()
            self._left_meas.popleft()
            self._right_set.popleft()
            self._right_meas.popleft()

    def _callback(self, msg: MotorState) -> None:
        now = time.time() - self._start_time
        with self._lock:
            self._t.append(now)
            self._left_set.append(msg.left_setpoint_rpm)
            self._left_meas.append(msg.left_measured_rpm)
            self._right_set.append(msg.right_setpoint_rpm)
            self._right_meas.append(msg.right_measured_rpm)
            self._trim(now)

    def snapshot(self):
        with self._lock:
            return (
                list(self._t),
                list(self._left_set),
                list(self._left_meas),
                list(self._right_set),
                list(self._right_meas),
            )


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot payload MotorState RPM traces")
    parser.add_argument(
        "--topic", default="/payload_0/motor_state", help="MotorState topic"
    )
    parser.add_argument(
        "--window-sec",
        type=float,
        default=20.0,
        help="Sliding plot window in seconds",
    )
    parser.add_argument(
        "--refresh-hz",
        type=float,
        default=20.0,
        help="Plot refresh rate",
    )
    args = parser.parse_args()

    rclpy.init()
    node = MotorStatePlotter(args.topic, args.window_sec)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.set_title("Payload Wheel RPM (Setpoint vs Measured)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("RPM")
    ax.grid(True, alpha=0.3)

    (left_set_line,) = ax.plot([], [], "b--", linewidth=1.5, label="left set")
    (left_meas_line,) = ax.plot([], [], "b", linewidth=2.0, label="left meas")
    (right_set_line,) = ax.plot([], [], "r--", linewidth=1.5, label="right set")
    (right_meas_line,) = ax.plot([], [], "r", linewidth=2.0, label="right meas")
    ax.legend(loc="upper right")

    def update(_frame):
        t, ls, lm, rs, rm = node.snapshot()
        if not t:
            return left_set_line, left_meas_line, right_set_line, right_meas_line

        left_set_line.set_data(t, ls)
        left_meas_line.set_data(t, lm)
        right_set_line.set_data(t, rs)
        right_meas_line.set_data(t, rm)

        ax.set_xlim(max(0.0, t[-1] - args.window_sec), max(args.window_sec, t[-1]))

        y_values = ls + lm + rs + rm
        y_min = min(y_values)
        y_max = max(y_values)
        pad = max(5.0, 0.1 * max(abs(y_min), abs(y_max), 1.0))
        ax.set_ylim(y_min - pad, y_max + pad)

        return left_set_line, left_meas_line, right_set_line, right_meas_line

    interval_ms = max(1, int(1000.0 / args.refresh_hz))
    fig._animation = FuncAnimation(fig, update, interval=interval_ms, blit=False)

    try:
        plt.show()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
