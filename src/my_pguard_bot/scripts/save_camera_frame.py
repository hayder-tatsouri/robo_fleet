#!/usr/bin/env python3
"""Subscribe to a sensor_msgs/Image topic, save first frame as PNG, exit.

Usage:
    ros2 run my_pguard_bot save_camera_frame \
        --ros-args -p topic:=/world_cam/chase -p out:=/tmp/chase.png -p timeout_s:=15.0
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


def image_msg_to_np(msg: Image) -> np.ndarray:
    """Minimal Image -> numpy converter (no cv_bridge)."""
    h, w = msg.height, msg.width
    enc = msg.encoding.lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8)

    if enc in ("rgb8", "bgr8"):
        img = buf.reshape((h, w, 3))
        if enc == "bgr8":
            img = img[:, :, ::-1]
    elif enc in ("rgba8", "bgra8"):
        img = buf.reshape((h, w, 4))
        if enc == "bgra8":
            img = img[:, :, [2, 1, 0, 3]]
    elif enc == "mono8":
        img = buf.reshape((h, w))
    else:
        raise ValueError(f"unsupported encoding {enc}")
    return img


class Saver(Node):
    def __init__(self) -> None:
        super().__init__("image_saver")
        self.declare_parameter("topic", "/world_cam/chase")
        self.declare_parameter("out", "/tmp/frame.png")
        self.declare_parameter("timeout_s", 20.0)
        self.declare_parameter("warmup_s", 2.0)

        self.topic = self.get_parameter("topic").value
        self.out = Path(self.get_parameter("out").value)
        self.timeout_s = float(self.get_parameter("timeout_s").value)
        self.warmup_s = float(self.get_parameter("warmup_s").value)

        self.done = False
        self.first_at: float | None = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub = self.create_subscription(Image, self.topic, self.cb, qos)
        self.get_logger().info(f"waiting for {self.topic} ...")

    def cb(self, msg: Image) -> None:
        if self.done:
            return
        now = time.monotonic()
        if self.first_at is None:
            self.first_at = now
            self.get_logger().info(
                f"first frame received ({msg.width}x{msg.height} {msg.encoding}); "
                f"waiting warmup {self.warmup_s:.1f}s")
            return
        if now - self.first_at < self.warmup_s:
            return

        try:
            img = image_msg_to_np(msg)
        except Exception as e:
            self.get_logger().error(f"decode failed: {e}")
            return

        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            self.out.parent.mkdir(parents=True, exist_ok=True)
            plt.imsave(str(self.out), img)
        except Exception as e:
            self.get_logger().error(f"save failed: {e}")
            return
        self.get_logger().info(f"wrote {self.out}")
        self.done = True


def main() -> int:
    rclpy.init()
    node = Saver()
    start = time.monotonic()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() - start > node.timeout_s:
                node.get_logger().error(
                    f"timeout after {node.timeout_s:.1f}s waiting for {node.topic}")
                node.destroy_node()
                rclpy.shutdown()
                return 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
