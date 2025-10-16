#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String

# for locating text display font file
from os import path

# Import all board pins.
from board import SCL, SDA
import busio

# Import the SSD1306 module.
import adafruit_ssd1306


class ssd1306_Node(Node):

    def __init__(self):
        # Create the I2C interface.
        self.i2c = busio.I2C(SCL, SDA)

        # Create the SSD1306 OLED class.
        # The first two parameters are the pixel width and pixel height.  Change these
        # to the right size for your display!
        self.display = adafruit_ssd1306.SSD1306_I2C(128, 32, self.i2c)

        # Clear the display.
        # Always call show after changing pixels to make the display update!
        self.display.fill(0)
        self.display.show()

        # Initialize ROS node
        super().__init__('lcd_ssd1306')
        # And add a string subscriber
        self.subscription = self.create_subscription(
            String,
            'display_text',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.display.fill(0) # clear display
        self.display.text(msg.data, 0, 0, 1, font_name=(path.dirname(path.realpath(__file__)) + '/font5x8.bin'))
        self.display.show()
        self.get_logger().info('Displaying: "%s"' % msg.data)


def main(args=None):
    try:
        rclpy.init(args=args)
        ssd1306_node = ssd1306_Node()

        rclpy.spin(ssd1306_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        ssd1306_node.destroy_node()


if __name__ == '__main__':
    main()
