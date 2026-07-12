#!/usr/bin/env python3
"""
Robo_Fleet <-> PGuard topic adapter.

Bridges the single-robot PGuard sim to the robo_fleet MCP server's expected
multi-robot topic layout.

Mapping:
  /odometry/filtered (nav_msgs/Odometry)  -->  /pguard/amcl_pose  (PoseWithCovarianceStamped)
  /pguard/cmd_vel   (TwistStamped)         -->  /cmd_vel           (Twist)
                                          -->  /pguard/battery_state  (BatteryState, synthetic 100%)
  /sonar/{front,left,rear,right} (Range)   -->  /pguard/scan       (LaserScan, 4-ray synth)

The Nav2 action `/navigate_to_pose` is also mirrored to `/pguard/navigate_to_pose`
via a relay so robo_fleet's `navigate_to_pose(robot_id='pguard', ...)` works
unchanged. Action relay uses a shim node that forwards the goal to the real
action server and pipes the result back.
"""

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.action.server import ServerGoalHandle, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist, TwistStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, LaserScan, Range
from nav2_msgs.action import NavigateToPose


class RoboFleetAdapter(Node):
    def __init__(self):
        super().__init__("robo_fleet_adapter")
        self._cb = ReentrantCallbackGroup()

        # Odometry -> amcl_pose (robo_fleet expects PoseWithCovarianceStamped)
        self.create_subscription(
            Odometry, "/odometry/filtered", self._on_odom, 10, callback_group=self._cb
        )
        self._pub_pose = self.create_publisher(
            PoseWithCovarianceStamped, "/pguard/amcl_pose", 10
        )

        # /pguard/cmd_vel (TwistStamped) -> /cmd_vel (Twist)
        self.create_subscription(
            TwistStamped, "/pguard/cmd_vel", self._on_cmd_vel_stamped, 10,
            callback_group=self._cb,
        )
        # Also accept plain Twist on /pguard/cmd_vel for robo_fleet's stop/emergency
        self.create_subscription(
            Twist, "/pguard/cmd_vel_twist", self._on_cmd_vel_twist, 10,
            callback_group=self._cb,
        )
        self._pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)

        # /sonar/front is already published as a LaserScan by gz-sim.
        # Just relay it under the /pguard/ namespace.
        self.create_subscription(
            LaserScan, "/sonar/front", self._on_scan, 10, callback_group=self._cb,
        )
        self._pub_scan = self.create_publisher(LaserScan, "/pguard/scan", 10)

        # Synthetic battery state @ 1 Hz
        self._pub_batt = self.create_publisher(BatteryState, "/pguard/battery_state", 10)
        self._battery_pct = 1.0
        self.create_timer(1.0, self._publish_battery, callback_group=self._cb)

        # Action relay: /pguard/navigate_to_pose -> /navigate_to_pose
        self._nav_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose", callback_group=self._cb
        )
        self._nav_server = ActionServer(
            self, NavigateToPose, "/pguard/navigate_to_pose",
            execute_callback=self._execute_nav,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=self._cb,
        )

        self.get_logger().info("robo_fleet_adapter running:")
        self.get_logger().info("  /odometry/filtered -> /pguard/amcl_pose")
        self.get_logger().info("  /pguard/cmd_vel(Stamped) -> /cmd_vel(Twist)")
        self.get_logger().info("  /sonar/* -> /pguard/scan (4-ray synth)")
        self.get_logger().info("  /pguard/battery_state @ 1Hz (100% synthetic)")
        self.get_logger().info("  /pguard/navigate_to_pose -> /navigate_to_pose (relay)")

    # ---- POSE ----
    def _on_odom(self, msg: Odometry):
        out = PoseWithCovarianceStamped()
        out.header = msg.header
        out.header.frame_id = "map"
        out.pose = msg.pose
        self._pub_pose.publish(out)

    # ---- CMD_VEL ----
    def _on_cmd_vel_stamped(self, msg: TwistStamped):
        self._pub_cmd.publish(msg.twist)

    def _on_cmd_vel_twist(self, msg: Twist):
        self._pub_cmd.publish(msg)

    # ---- SONAR -> SCAN ----
    def _on_scan(self, msg: LaserScan):
        self._pub_scan.publish(msg)

    # ---- BATTERY ----
    def _publish_battery(self):
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = self._battery_pct
        msg.voltage = 10.0 + self._battery_pct * 2.6
        msg.current = -0.5
        msg.present = True
        self._pub_batt.publish(msg)

    # ---- NAV2 RELAY ----
    async def _execute_nav(self, goal_handle: ServerGoalHandle):
        self.get_logger().info(
            f"Nav goal received: ({goal_handle.request.pose.pose.position.x:.2f}, "
            f"{goal_handle.request.pose.pose.position.y:.2f})"
        )
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            goal_handle.abort()
            return NavigateToPose.Result()

        send_future = self._nav_client.send_goal_async(
            goal_handle.request,
            feedback_callback=lambda fb: goal_handle.publish_feedback(fb.feedback),
        )
        client_goal = await send_future
        if not client_goal.accepted:
            goal_handle.abort()
            return NavigateToPose.Result()

        result_future = client_goal.get_result_async()
        result_wrapped = await result_future
        goal_handle.succeed()
        return result_wrapped.result


def main():
    rclpy.init()
    node = RoboFleetAdapter()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
