#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time
import math

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(TransformStamped, '/ciao', 10)
        self.publisher2_ = self.create_publisher(TransformStamped, '/come', 10)
        
        self.timer = self.create_timer(0.01, self.publish_pose)

    def publish_pose(self):
        pi_h = math.sqrt(2.) / 2.

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()  # Autofill current time
        msg.header.frame_id = 'ciao'
        msg.child_frame_id = 'come'
        msg.transform.translation.x = 1.0
        msg.transform.translation.y = 0.0
        msg.transform.translation.z = 0.0
        msg.transform.rotation.x = 0.0
        msg.transform.rotation.y = 0.
        msg.transform.rotation.z = pi_h
        msg.transform.rotation.w = pi_h

        self.publisher_.publish(msg)
        msg2 = msg
        msg.header.frame_id = 'come'
        msg.child_frame_id = 'va'
        msg.transform.translation.x = 2.0
        msg.transform.translation.y = 0.0
        msg.transform.translation.z = 0.0
        msg2.transform.rotation.x = 0.
        msg2.transform.rotation.y = pi_h
        msg2.transform.rotation.z = 0.
        msg2.transform.rotation.w = pi_h

        self.publisher2_.publish(msg2)
        self.get_logger().info('Publishing PoseStamped with current timestamp')

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()