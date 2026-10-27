"""Run a ROS 2 node until Ctrl-C or SIGTERM, then shut down cleanly (shared main() for every node)."""

from __future__ import annotations

from collections.abc import Callable

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def run(make_node: Callable[[], Node]) -> None:
    rclpy.init()
    node = make_node()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl-C, or SIGTERM from the launch scripts' stop_all
    except Exception:
        # SIGTERM can invalidate the context mid-spin (RCLError "context is not valid"); only real errors propagate.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
