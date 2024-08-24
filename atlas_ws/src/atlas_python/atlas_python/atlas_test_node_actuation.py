import sys
import rclpy
from rclpy.node import Node

try:
    import RPi.GPIO as GPIO
except RuntimeError: # RPi.GPIO throws errors when not on RPi
    from unittest.mock import MagicMock as GPIO

import time
import os
import signal
import subprocess
import sys
import threading

# Messages
from std_msgs.msg import String
from atlas_msgs.msg import DiffDriveMotorVelocities
from atlas_msgs.msg import DiffDriveEncoderReadings
from atlas_msgs.msg import DiffDriveMotorPWMCommand

class testNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.get_logger("test_node initiated")

        # Publisher
        self.publisher_ = self.create_publisher(DiffDriveMotorPWMCommand, 'motor_command', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publisher_callback)
        self.i = 0

        # Subscriptions
        self.enc_subscription = self.create_subscription(DiffDriveEncoderReadings, "encoderReads_pertick", self.encoders_subscriptions_callback, 10)
        self.motors_subscriptions = self.create_subscription(DiffDriveMotorVelocities, "Motor Velocities", self.motors_subscriptions_callback, 10)


    def publisher_callback(self):
        msg = DiffDriveMotorPWMCommand
        msg.is_pwm = True
        msg.l_motor_duty_cycle = 20
        msg.l_motor_forward = True
        msg.r_motor_duty_cycle = 20
        msg.r_motor_forward = False

        self.publisher_.publish(msg)
        self.get_logger().info("Publishing")


    def encoders_subscriptions_callback(self, msg):
        self.get_logger().info('I heard: {} and {}'.format(msg.l_motor_encoder_val, msg.r_motor_encoder_val))

    def motors_subscriptions_callback(self, msg):
        self.get_logger().info('I heard: {} and {}'.format(msg.l_motor_w, msg.r_motor_w))

def main(args=None):
    rclpy.init(args=args)

    test_node = testNode()

    rclpy.spin(test_node)

    test_node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
