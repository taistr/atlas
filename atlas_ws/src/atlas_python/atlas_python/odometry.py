# RUN THIS AS A BACKGROUND PROCESS

# ROS 2 Set ups
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


try:
    import RPi.GPIO as GPIO
except RuntimeError: # RPi.GPIO throws errors when not on RPi
    from unittest.mock import MagicMock as GPIO

import RPi.GPIO
import time
import os
import signal
import subprocess
import sys
import threading
from math import sin, cos, pi

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Int16

NS_TO_SEC= 1000000000

# Raspberry Pi GPIO settings
#Left Motor Encoder inputs
GPIO_LA = 2
GPIO_LB = 3
#Right Motor Encoder inputs
GPIO_RA = 4
GPIO_RB = 5

# Define Tick Rate for motor drives - Anh
TICK_RATE = 100 #100Hz

# Robot Params
WHEEL_OD = 56/1000 #meters

## NEED TO CHANGE
WHEEL_BASELINE = 10/100 #meters
##################################


# Source 1: https://gist.github.com/savetheclocktower/9b5f67c20f6c04e65ed88f2e594d43c1?permalink_comment_id=3628601
# Source 2: Google Gemini
# Source 3: https://github.com/eden-desta/ros2_differential_drive/blob/ros2/src/differential_drive/diff_tf.py
class odometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        # GPIO Set Up
        self.levLA = 0
        self.levLB = 0
        self.levRA = 0
        self.levRB = 0

        self.lastGpio = None
        self.gpioLA = GPIO_LA
        self.gpioLB = GPIO_LB
        self.gpioRA = GPIO_RA
        self.gpioRB = GPIO_RB

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.gpioLA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.gpioLB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.gpioRA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.gpioRB, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        GPIO.add_event_detect(self.gpioLA, GPIO.BOTH, self.Lencoder_callback)
        GPIO.add_event_detect(self.gpioLB, GPIO.BOTH, self.Lencoder_callback)
        GPIO.add_event_detect(self.gpioRA, GPIO.BOTH, self.Rencoder_callback)
        GPIO.add_event_detect(self.gpioRB, GPIO.BOTH, self.Rencoder_callback)

        # Encoder decoder set up
        self.Lcounter = 0
        self.Rcounter = 0
        self.Llast_counter = 0
        self.Rlast_counter = 0
        self.last_publish_time = 0

        # Odometry data set up
        self.x = 0.0  # position in xy plane
        self.y = 0.0
        self.th = 0.0
        self.dx = 0.0  # speeds in x/rotation
        self.dr = 0.0
        self.base_frame_id = self.declare_parameter('base_frame_id', 'base_link').value # robot's base frame
        self.odom_frame_id = self.declare_parameter('odom_frame_id', 'odom').value  # odometry reference frame

        # Publisher set up
        self.publisher_ = self.create_publisher(Float32MultiArray, "odometry_state_vector_pertick", queue_size = 10)
    
        # Publishing Timer
        self.timer = self.create_timer(1/TICK_RATE, self.periodicPublisher)

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
        time_elapsed = time_elapsed.nanoseconds / 1000000000
        ticks_elapsed = time_elapsed/(1/TICK_RATE)

        # Calculate counts per tick
        if ticks_elapsed > 0:
            Lcounts_perTick = (self.Lcounter - self.Llast_counter) / ticks_elapsed
            Rcounts_perTick = (self.Rcounter - self.Rlast_counter) / ticks_elapsed
        else:
            print("Tick period = 0")

        # Convert counts/tick to m/tick
        Lw = Lcounts_perTick * (360/48) * (1/74.8317)       #Counts/Tick * Angle/Count * Exact Gear Ratio = Angle/Tick * Exact Gear Ratio (Degrees/Tick)
        Lv = (Lw/360) * (pi*WHEEL_OD)      # meter/tick
        Rw = Rcounts_perTick * (360/48) * (1/74.8317)       #Counts/Tick * Angle/Count * Exact Gear Ratio = Angle/Tick * Exact Gear Ratio (Degrees/Tick)
        Rv = (Rw/360) * (pi*WHEEL_OD)      # meter/tick

        # Displacement/Tick
        #Polar coords
        d_perTick = (Rv + Lv) / 2     #meter/Tick
        th_perTick = (Rv - Lv) / WHEEL_BASELINE      #Degrees/Tick
        #Cartesian coords
        if Lv != 0:
            x_perTick = cos(th_perTick) * d_perTick
            y_perTick = -sin(th_perTick) * d_perTick
            self.x = self.x + (cos(self.th) * x_perTick - sin(self.th)*y_perTick)
            self.y = self.y + (sin(self.th) * x_perTick + cos(self.th)*y_perTick)
        if th_perTick != 0:
            self.th = self.th + th_perTick 
        #Quaternion
        """
        If you're curious about the values in a quaternion:
            If v(v1, v2, v3) is the axis of rotation:
             x = v1 * sin (theta / 2)
             y = v2 * sin (theta / 2)
             z = v3 * sin (theta / 2)
            w represents the "amount" of rotation:
             w = cos(theta/2)
        """
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin(self.th/2)
        quaternion.w = cos(self.th/2)

        # Velocities 
        self.dx = d_perTick / time_elapsed   #meter/second
        self.dr = th_perTick / time_elapsed  #degrees/second

        # Odometry message
        msg = Odometry()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = self.odom_frame_id
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = quaternion
        msg.child_frame_id = self.base_frame_id
        msg.twist.twist.linear.x = self.dx
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.angular.z = self.dr

        #Publish information
        self.publisher_.publish(msg)

        #Reset counter values
        self.Llast_counter = self.Lcounter
        self.Rlast_counter = self.Rcounter
        self.last_publish_time = current_time    
        

    def destroy(self):
        GPIO.remove_event_detect(self.gpioLA)
        GPIO.remove_event_detect(self.gpioLB)
        GPIO.remove_event_detect(self.gpioRA)
        GPIO.remove_event_detect(self.gpioRB)
        GPIO.cleanup()


class encoderError(Exception):
    print("Encoder Error Occurred. Bypassing Error")
    pass


def main():
    rclpy.init(args = args)
    try:
        odometry_node = odometryNode()
        rclpy.spin(odometry_node)
    except rclpy.exceptions.ROSInterruptException:
        pass

    odometry_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":  
    main()
   



