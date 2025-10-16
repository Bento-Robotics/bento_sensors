#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String

from demo_sensor_modules.demo_sensor_lib import data


class DemoSensor(Node):

    def __init__(self):
        super().__init__('demo_sensor')
        self.publisher_ = self.create_publisher(String, 'data', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = data
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    try:
        rclpy.init(args=args)
        sensor_demo = DemoSensor()

        rclpy.spin(sensor_demo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        sensor_demo.destroy_node()


if __name__ == '__main__':
    main()
