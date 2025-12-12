#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/trajectory_controller/joint_trajectory',
            10
        )

        # Timer: publish every 2 seconds
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_trajectory)
        self.angle = 0.0  # Initial position

    def publish_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = ['joint_1', 'joint_2']

        # Increment the angle each publish
        self.angle += 5000.0
        if self.angle > 20000.0:  # Optional: reset after some max
            self.angle = 0.0

        point = JointTrajectoryPoint()
        point.positions = [self.angle, self.angle]  # same for both joints
        point.time_from_start.sec = 2
        point.time_from_start.nanosec = 0

        msg.points.append(point)

        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing positions: {point.positions}')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
