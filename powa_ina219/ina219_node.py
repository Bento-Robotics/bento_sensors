#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import BatteryState

# For float 'NaN'
import math

# Import all board pins.
from board import SCL, SDA
import busio

from adafruit_ina219 import INA219, ADCResolution


class ina219_Node(Node):

    def __init__(self):
        # Create the I2C interface.
        self.i2c = busio.I2C(SCL, SDA)
        # Set up INA219 sensor
        self.sensor = INA219(self.i2c)
        # Configure INA219 to only take 2 samples per measurement
        self.sensor.bus_adc_resolution = ADCResolution.ADCRES_12BIT_2S
        self.sensor.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_2S

        # Initialize ROS node
        super().__init__('powa_ina219')
        # And add a string subscriber
        self.publisher_ = self.create_publisher(BatteryState, 'battery', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = BatteryState()
        # Stuff we don't measure - ROS standards say these should all be NaN
        msg.charge = msg.capacity = msg.design_capacity = math.nan
        msg.cell_voltage = [math.nan] * 6
        # This battery (should) always be present
        msg.present = True
        # This is a Li-Ion battery
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        # Guessing things are fine.
        # If something interesting is going on, we change it later. This is the default.
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD

        # Voltage
        msg.voltage = float(f'{self.sensor.bus_voltage:.2f}')
        # Current - inverted: "as seen from battery"
                        # cut all decimal digits, as they are mostly noise
        msg.current = - float(f'{(self.sensor.current/1000):.0f}')
        # Percentage 0~1 - 6s battery, each 3.0V ~ 4.2V
        msg.percentage = float(f'{(msg.voltage - 3.0*6) * 100 / (4.2*6 - 3.0*6):.0f}')

        if msg.voltage < 1.1 and msg.voltage > 0.7:
            self.get_logger().warn('This is around the measuring voltage if the INA219 has nothing connected. You may have a floating ground connection.')
        if msg.voltage < (6 * 3.0): # lower limit
            self.get_logger().error('Battery Empty!')
        if msg.voltage > (6 * 4.2): # upper limit
            self.get_logger().error('Battery Overcharged!')
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_OVERVOLTAGE
        if msg.current > 0: # current flowing into battery
            self.get_logger().info('Battery Charging.')
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING

        self.publisher_.publish(msg)


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
