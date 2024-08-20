# RUN THIS AS A BACKGROUND PROCESS

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

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Int16
from atlas_msgs.msg import DiffDriveMotorVelocities
from atlas_msgs.msg import DiffDriveEncoderReadings

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
        self.name = "odometry"
        self.get_logger().info("Odometry Node Online!")
       
        # Odometry data set up
        self.x = 0.0  # position in xy plane
        self.y = 0.0
        self.th = 0.0
        self.dx = 0.0  # speeds in x/rotation
        self.dr = 0.0
        self.base_frame_id = self.declare_parameter('base_frame_id', 'base_link').value # robot's base frame
        self.odom_frame_id = self.declare_parameter('odom_frame_id', 'odom').value  # odometry reference frame

        # Subscription set up
        self.subscription = self.create_subscription(DiffDriveEncoderReadings,'encReading_perTick',self.periodicPublisher,10)

        # Publisher set up
        self.publisher_ = self.create_publisher(Odometry, "odometry_pertick", queue_size = 10)
    
        # Publishing Timer
        self.timer = self.create_timer(1/TICK_RATE, self.periodicPublisher)


    def periodicPublisher(self, encReading_perTick):
        # Calculate elapsed time
        current_time = self.get_clock().now()
        time_elapsed = current_time - self.last_publish_time
        time_elapsed = time_elapsed.nanoseconds / NS_TO_SEC
        ticks_elapsed = time_elapsed/(1/TICK_RATE)

        # Calculate counts per tick
        if ticks_elapsed > 0:
            Lcounts_perTick = encReading_perTick.l_motor_encoder_val
            Rcounts_perTick = encReading_perTick.r_motor_encoder_val
        else:
            print("Odometry: Time period = 0")

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
    

class encoderError(Exception):
    print("Encoder Error Occurred. Bypassing Error")
    pass


def main(args=None):
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
   



