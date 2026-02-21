#!/usr/bin/env python3
"""Failsafe trigger daemon. Listens on a Unix socket; when triggered, calls /mode_manager/failsafe immediately."""

import os
import socket

import rclpy
from std_srvs.srv import Trigger

SOCKET_PATH = "/tmp/penn_failsafe"


def main():
    rclpy.init()
    node = rclpy.create_node("failsafe_trigger_daemon")
    client = node.create_client(Trigger, "/mode_manager/failsafe")

    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Waiting for /mode_manager/failsafe...")

    if os.path.exists(SOCKET_PATH):
        os.unlink(SOCKET_PATH)
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.bind(SOCKET_PATH)
    sock.listen(1)
    os.chmod(SOCKET_PATH, 0o777)
    node.get_logger().info(f"Listening on {SOCKET_PATH}")

    while rclpy.ok():
        try:
            conn, _ = sock.accept()
            try:
                conn.recv(64)
                req = Trigger.Request()
                future = client.call_async(req)
                rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
                if future.result() is not None:
                    node.get_logger().info("Failsafe triggered via daemon")
            except Exception as e:
                node.get_logger().error(f"Failsafe trigger failed: {e}")
            finally:
                conn.close()
        except Exception as e:
            node.get_logger().error(f"Socket accept error: {e}")

    sock.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
