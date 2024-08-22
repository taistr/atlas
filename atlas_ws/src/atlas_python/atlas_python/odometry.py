import sys
from dataclasses import dataclass
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from atlas_msgs.msg import EncoderCount, WheelVelocity
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler


@dataclass
class Pose:
    """Dataclass to store Atlas' pose"""
    x: float = 0.0 # position in xy plane
    y: float = 0.0
    theta: float = 0.0 # orientation
    v: float = 0.0 # linear velocity
    w: float = 0.0 # angular velocity

@dataclass
class WheelVelocity:
    """Dataclass to store Atlas' wheel velocities"""
    left: float = 0.0
    right: float = 0.0

class OdometryNode(Node):
    def __init__(self):
        super().__init__("odometry")
        self.initialise_parameters()

        # Set up variables
        self.pose = Pose()
        self.wheel_velocities = WheelVelocity()
        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_base = self.get_parameter("wheel_base").get_parameter_value().double_value
        self.counts_per_rev = self.get_parameter("counts_per_rev").get_parameter_value().integer_value
        self.prev_encoder_msg: EncoderCount | None = None

        # Set up ROS 2 interfaces
        odometry_publish_rate = 1.0/self.get_parameter("publish_rate").get_parameter_value().double_value
        self.odometry_update_timer = self.create_timer(odometry_publish_rate, self.odometry_callback)
        self.encoder_subcriber = self.create_subscription(
            EncoderCount, 
            "atlas/encoder_count", 
            self.encoder_callback, 
            qos_profile=QoSProfile(
                depth=10,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.BEST_EFFORT
            )
        )
        self.odometry_publisher = self.create_publisher(
            Odometry, 
            "atlas/odometry", 
            qos_profile=QoSProfile(
                depth=10,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.BEST_EFFORT
            )
        )

        self.get_logger().info("Odometry Node Online!")

    def initialise_parameters(self) -> None:
        """Declare parameters for the encoder node"""
        self.declare_parameter(
            "publish_rate",
            value=50,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Rate at which odometry data is published (Hz)"
            )
        )
        self.declare_parameter(
            "counts_per_rev",
            value=48,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="Number of encoder counts per revolution"
            )
        )
        self.declare_parameter(
            "wheel_base",
            value=0.1,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Distance between the two wheels of the robot (m)"
            )
        )
        self.declare_parameter(
            "wheel_radius",
            value=0.05,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Radius of the wheels of the robot (m)"
            )
        )

    def update_pose(self, msg: EncoderCount) -> None:
        circumference = 2 * math.pi * self.wheel_radius
        delta_s_left = circumference * ((msg.left - self.prev_encoder_msg.left) / self.counts_per_rev)
        delta_s_right = circumference * ((msg.right - self.prev_encoder_msg.right) / self.counts_per_rev)

        delta_s = (delta_s_left + delta_s_right) / 2
        delta_theta = (delta_s_right - delta_s_left) / self.wheel_base

        self.pose.x += delta_s * math.cos(self.pose.theta + delta_theta / 2.0)
        self.pose.y += delta_s * math.sin(self.pose.theta + delta_theta / 2.0)
        self.pose.theta += delta_theta

        delta_t = (msg.time - self.prev_encoder_msg.time) / 1e9 # seconds
        self.pose.v = delta_s / delta_t
        self.pose.w = delta_theta / delta_t

    def encoder_callback(self, msg: EncoderCount) -> None:
        """Callback function for the encoder data. Updates the odometry data"""
        if self.prev_encoder_msg is None:
            self.prev_encoder_msg = msg
            return
        
        self.update_pose(msg)
        self.prev_encoder_msg = msg

    def get_odometry(self) -> Odometry:
        """Get the current odometry data"""
        msg: Odometry = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        # Set the position
        msg.pose.pose.position.x = self.pose.x
        msg.pose.pose.position.y = self.pose.y
        msg.pose.pose.position.z = 0.0

        # Set the orientation (quaternion)
        quaternion = quaternion_from_euler(0, 0, self.pose.theta)
        msg.pose.pose.orientation = Quaternion(*quaternion)

        # Set the velocity
        msg.twist.twist.linear.x = self.pose.v
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = self.pose.w

        return msg

    def odometry_callback(self) -> None:
        """Publish the odometry data"""

        #Publish the Wheel Velocities

        # Publish the odometry message
        msg = self.get_odometry()
        self.odometry_publisher.publish(msg)

    def timer_callback(self):
        self.get_logger().info("callback worked")
    

def main(args: dict = None):
    rclpy.init(args=args)
    
    odometry = OdometryNode()
    try:
        odometry.get_logger().info("Starting Actuation Test!")
        rclpy.spin(odometry, MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass
    except rclpy.exceptions.ExternalShutdownException:
        sys.exit(1)
    finally:
        odometry.cleanup()  # Ensure GPIO cleanup happens
        odometry.destroy_node()

    rclpy.shutdown()
