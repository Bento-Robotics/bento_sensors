#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import Temperature



# For float 'NaN'
import math

# Import all board pins.
#from board import SCL, SDA
#import busio
#from mpu9250_jmdev.registers import *
#from mpu9250_jmdev.mpu_9250 import MPU9250



class mpu9250_Node(Node):
    def __init__(self):
        # Create the I2C interface.
       # self.i2c = busio.I2C(SCL, SDA)
        # Set up MPu8 sensor
       # self.sensor = MPU9250(self.i2c)
        #self.sensor.configure()  # Apply the settings to the registers.


        # Initialize ROS node
        super().__init__('imu_mpu9250')
        # And add a string subscriber
        self.publisher_imu = self.create_publisher(Imu, "imu", 10)
        self.publisher_mag = self.create_publisher(MagneticField, "mag", 10)
        self.publisher_temp = self.create_publisher(Temperature, "temp", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg_imu = Imu()
        msg_mag = MagneticField()
        msg_temp = Temperature()

        msg_imu.header.stamp = self.get_clock().now().to_msg()
        msg_mag.header.stamp = self.get_clock().now().to_msg()
        msg_temp.header.stamp = self.get_clock().now().to_msg()

        msg_imu.angular_velocity.x = 1.0
        msg_imu.angular_velocity.y = 2.0
        msg_imu.angular_velocity.z = 3.0
        msg_imu.linear_acceleration.x = 4.0
        msg_imu.linear_acceleration.y = 5.0
        msg_imu.linear_acceleration.z = 6.0
        msg_imu.quaternion.x = 7.0
        msg_imu.quaternion.y = 8.0
        msg_imu.quaternion.z = 9.0
        msg_mag.magnetic_field.x = 10.0
        msg_mag.magnetic_field.y = 11.0
        msg_mag.magnetic_field.z = 12.0


        # Variance from datasheet ≈ ±3%
        # We're ignoring covariance (due to knowledge limitations) in the matrix
        # ┌ 0 1 2 ┐   ┌ x ┐
        # │ 3 4 5 │ * │ y │
        # └ 6 7 8 ┘   └ z ┘
        msg_imu.angular_velocity_covariance[0] = msg_imu.angular_velocity.x * 0.031
        msg_imu.angular_velocity_covariance[4] = msg_imu.angular_velocity.y * 0.031
        msg_imu.angular_velocity_covariance[8] = msg_imu.angular_velocity.z * 0.031
        msg_imu.linear_acceleration_covariance[0] = msg_imu.linear_acceleration.x * 0.035
        msg_imu.linear_acceleration_covariance[4] = msg_imu.linear_acceleration.y * 0.035
        msg_imu.linear_acceleration_covariance[8] = msg_imu.linear_acceleration.z * 0.035
        msg_imu.quaternion_covariance[0] = msg_imu.quaternion.x
        msg_imu.quaternion_covariance[4] = msg_imu.quaternion.y
        msg_imu.quaternion_covariance[8] = msg_imu.quaternion.z
        msg_mag.magnetic_field_covariance[0] = msg_mag.magnetic_field.x * 0.06
        msg_mag.magnetic_field_covariance[4] = msg_mag.magnetic_field.y * 0.06
        msg_mag.magnetic_field_covariance[8] = msg_mag.magnetic_field.z * 0.06
        msg_temp.temperature.variance = msg_temp.temperature * 0.041
        self.publisher_imu.publish(msg_imu)
        self.publisher_imu.publish(msg_temp)
        self.publisher_mag.publish(msg_mag)



def main(args=None):
    try:
        rclpy.init(args=args)
        mpu9250_node = mpu9250_Node()

        rclpy.spin(mpu9250_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        mpu9250_node.destroy_node()


if __name__ == '__main__':
    main()
