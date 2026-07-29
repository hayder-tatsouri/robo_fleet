#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class GpsTfPublisher(Node):
    def __init__(self):
        super().__init__('gps_tf_publisher')
        self.br = TransformBroadcaster(self)
        self.declare_parameter('parent_frame', 'base_footprint')
        self.declare_parameter('child_frame', 'base_footprint/navsat_sensor')
        self.timer = self.create_timer(0.5, self.publish)

    def publish(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.get_parameter('parent_frame').value
        t.child_frame_id = self.get_parameter('child_frame').value
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)


def main():
    import sys
    rclpy.init(args=sys.argv)
    node = GpsTfPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
