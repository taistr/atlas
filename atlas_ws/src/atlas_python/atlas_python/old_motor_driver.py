#!/usr/bin/env python
#Source: https://github.com/joshnewans/serial_motor_demo/blob/main/serial_motor_demo/serial_motor_demo/driver.py

# ROS 2 Set ups
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
from math import sin, cos, pi

from tf2_ros import TransformBroadcaster
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16
from atlas_msgs.msg import DiffDriveMotorVelocities
from atlas_msgs.msg import DiffDriveEncoderReadings
from atlas_msgs.msg import DiffDriveMotorPWMCommand


NS_TO_SEC= 1000000000

# Raspberry Pi GPIO settings
#Left Motor Encoder inputs
GPIO_LA = 2
GPIO_LB = 3
#Left Moto PWM outputs
IN1 = 14
IN2 = 15
ENL = 18  # PWM pin for left motor 

#Right Motor Encoder inputs
GPIO_RA = 4
GPIO_RB = 5
# Define pins for motor B
IN3 = 27
IN4 = 22
ENR = 17  # PWM pin for right motor 

# Define Tick Rate for motor drives - Anh
TICK_RATE = 100 #100Hz

# Robot Params
WHEEL_OD = 56/1000 #meters
MOTOR_GEAR_MULTIPLIER = 74.8317
ENCODER_PPR = 48

class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')

        self.name = "motor_driver"
        self.get_logger().info("Motor Driver Interface Online!")\
        
        # GPIO Set Up
        #Encoders
        GPIO.setmode(GPIO.BCM)

        self.levLA = 0
        self.levLB = 0
        self.levRA = 0
        self.levRB = 0

        self.lastGpio = None
        self.gpioLA = GPIO_LA
        self.gpioLB = GPIO_LB
        self.gpioRA = GPIO_RA
        self.gpioRB = GPIO_RB

        GPIO.setup(self.gpioLA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.gpioLB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.gpioRA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.gpioRB, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        GPIO.add_event_detect(self.gpioLA, GPIO.BOTH, self.Lencoder_callback)
        GPIO.add_event_detect(self.gpioLB, GPIO.BOTH, self.Lencoder_callback)
        GPIO.add_event_detect(self.gpioRA, GPIO.BOTH, self.Rencoder_callback)
        GPIO.add_event_detect(self.gpioRB, GPIO.BOTH, self.Rencoder_callback)

        #Motor PWM
        # Set up pins for left motor
        GPIO.setup(ENL, GPIO.OUT)
        GPIO.setup(IN1, GPIO.OUT)
        GPIO.setup(IN2, GPIO.OUT)
        self.leftMotorDirectionPins = {'forward': IN1, 'reverse':IN2}
        # Set up pins for right motor 
        GPIO.setup(ENR, GPIO.OUT)
        GPIO.setup(IN3, GPIO.OUT)
        GPIO.setup(IN4, GPIO.OUT)
        self.rightMotorDirectionPins = {'forward': IN3, 'reverse':IN4}
        # Set up PWM for left motor 
        self.pwm_L = GPIO.PWM(ENL, TICK_RATE)  # PWM frequency set to 100 Hz
        self.pwm_L.start(0)  # Start PWM with 0% duty cycle
        # Set up PWM for right motor
        self.pwm_R = GPIO.PWM(ENR, TICK_RATE)  # PWM frequency set to 100 Hz
        self.pwm_R.start(0)  # Start PWM with 0% duty cycle

        # Encoder decoder set up
        self.Lcounter = 0
        self.Rcounter = 0
        self.Llast_counter = 0
        self.Rlast_counter = 0
        self.last_publish_time = 0
        
        # Subscription set up
        self.subscription = self.create_subscription(DiffDriveMotorPWMCommand,'motor_command',self.motor_command_callback,10)

        # Publisher set up
        self.velPublisher = self.create_publisher(DiffDriveMotorVelocities, "Motor Velocities", queue_size = 10)
        self.encPublisher = self.create_publisher(DiffDriveEncoderReadings, "encoderReads_pertick", queue_size = 10)
    
        # Publishing Timer -UNUSED
        #self.timer = self.create_timer(1/TICK_RATE, self.periodicPublisher)

    def Lencoder_callback(self, channel):
        # Checking the most recent pin set to "high"
        Llevel = GPIO.input(channel)
        if channel == self.gpioLA:
            self.levLA = Llevel
        else:
            self.levLB = Llevel
    
        # Debounce
        if channel == self.lastGpio:
            return
    
        # When both inputs are at 1, we'll fire a callback. If A was the most
        # recent pin set high, it'll be forward, and if B was the most recent pin
        # set high, it'll be reverse.
        self.lastGpio = channel
        if channel == self.gpioLA and Llevel == 1:    #Forward
            if self.levLB == 1:
                self.Lcounter += 1
        elif channel == self.gpioLB and Llevel == 1:  #Reverse
            if self.levLA == 1:
                self.Lcounter -= 1


    def Rencoder_callback(self, channel):
        # Checking the most recent pin set to "high"
        Rlevel = GPIO.input(channel)
        if channel == self.gpioRA:
            self.levRA = Rlevel
        else:
            self.levRB = Rlevel
    
        # Debounce
        if channel == self.lastGpio:
            return
    
        # When both inputs are at 1, we'll fire a callback. If A was the most
        # recent pin set high, it'll be forward, and if B was the most recent pin
        # set high, it'll be reverse.
        self.lastGpio = channel
        if channel == self.gpioRA and Rlevel == 1:    #Forward
            if self.levRB == 1:
                self.Rcounter += 1
        elif channel == self.gpioRB and Rlevel == 1:  #Reverse
            if self.levRA == 1:
                self.Rcounter -= 1


    def periodicPublisher(self):
        # Calculate elapsed time
        current_time = self.get_clock().now()
        time_elapsed = current_time - self.last_publish_time
        time_elapsed = time_elapsed.nanoseconds / NS_TO_SEC
        ticks_elapsed = time_elapsed/(1/TICK_RATE)

        # Calculate counts per tick
        if time_elapsed > 0:
            Lcounts_perTick = (self.Lcounter - self.Llast_counter)
            Rcounts_perTick = (self.Rcounter - self.Rlast_counter)
        else:
            print("Time period = 0")

        # Convert counts/tick to degrees/second
        Lw = Lcounts_perTick * (360/ENCODER_PPR) * (1/MOTOR_GEAR_MULTIPLIER) / time_elapsed       #Counts/Tick * Angle/Count * Exact Gear Ratio / Time Elapsed = (Degrees/Second)
        Rw = Rcounts_perTick * (360/ENCODER_PPR) * (1/MOTOR_GEAR_MULTIPLIER) / time_elapsed      #Counts/Tick * Angle/Count * Exact Gear Ratio / Time Elapsed = (Degrees/Second)
        
        vel_msg = DiffDriveMotorVelocities()
        vel_msg.l_motor_w = Lw
        vel_msg.r_motor_w = Rw
        self.velPublisher.publish(vel_msg)

        enc_msg = DiffDriveEncoderReadings()
        enc_msg.l_motor_encoder_val = Lcounts_perTick
        enc_msg.r_motor_encoder_val = Rcounts_perTick
        self.encPublisher.publish(enc_msg)
        
        #Reset counter values
        self.Llast_counter = self.Lcounter
        self.Rlast_counter = self.Rcounter
        self.last_publish_time = current_time    


    def set_motor_speed(self, pwm, speed):
        pwm.ChangeDutyCycle(speed)


    def set_motor_direction(self, in1, in2, direction):
        if direction == "forward":
            GPIO.output(in1, GPIO.HIGH)
            GPIO.output(in2, GPIO.LOW)
        elif direction == "reverse":
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.HIGH)
        else:
            return


    def motor_command_callback(self, motor_command):
        if (motor_command.is_pwm):
            # Left motor
            if (motor_command.l_motor_forward):
                self.set_motor_direction(self.leftMotorDirectionPins['forward'],self.leftMotorDirectionPins['reverse'], "forward")
                self.set_motor_speed(self.pwm_L, motor_command.l_motor_duty_cycle)
            elif (~motor_command.l_motor_forward):
                self.set_motor_direction(self.leftMotorDirectionPins['forward'],self.leftMotorDirectionPins['reverse'], "reverse")
                self.set_motor_speed(self.pwm_L, motor_command.l_motor_duty_cycle)

            # Right motor
            if (motor_command.r_motor_forward):
                self.set_motor_direction(self.rightMotorDirectionPins['forward'],self.rightMotorDirectionPins['reverse'], "forward")
                self.set_motor_speed(self.pwm_R, motor_command.r_motor_duty_cycle)
            elif (~motor_command.r_motor_forward):
                self.set_motor_direction(self.rightMotorDirectionPins['forward'],self.rightMotorDirectionPins['reverse'], "reverse")
                self.set_motor_speed(self.pwm_R, motor_command.r_motor_duty_cycle)
        else:
            print("Motor command is not PWM.")
            pass
    
    def motorCleanUp(self):
        # Cleanup
        self.pwm_L.stop()
        self.pwm_R.stop()
        GPIO.remove_event_detect(self.gpioLA)
        GPIO.remove_event_detect(self.gpioLB)
        GPIO.remove_event_detect(self.gpioRA)
        GPIO.remove_event_detect(self.gpioRB)
        GPIO.cleanup()
        GPIO.cleanup()


def main(args = None):
    rclpy.init(args=args)

    motor_driver = MotorDriver()

    rate = motor_driver.create_rate(TICK_RATE)
    while rclpy.ok():
        rclpy.spin_once(motor_driver)
        motor_driver.periodicPublisher

    motor_driver.motorCleanUp()
    motor_driver.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()