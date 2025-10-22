#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import BatteryState

# for locating text display font file
from os import path

# Import all board pins.
from board import SCL, SDA
import busio

import adafruit_ina219


class ina219_Node(Node):

    def __init__(self):
        # Create the I2C interface.
        self.i2c = busio.I2C(SCL, SDA)

        self.sensor = adafruit_ina219.INA219(self.i2c)


        # Initialize ROS node
        super().__init__('powa_ina219')
        # And add a string subscriber
        self.publisher_ = self.create_publisher(BatteryState, 'battery', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = BatteryState()
        msg.voltage = float(self.sensor.bus_voltage)
        msg.current = float(self.sensor.current)
        # 6s battery, each 3.0V ~ 4.2V
        msg.percentage = float( (msg.voltage - 3.0*6) * (100 - 0) / (4.2*6 - 3.0*6) + 0 )
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing voltage')


def main(args=None):
    try:
        rclpy.init(args=args)
        ina219_node = ina219_Node()

        rclpy.spin(ina219_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        ina219_node.destroy_node()


if __name__ == '__main__':
    main()
