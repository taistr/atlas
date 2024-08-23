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
from atlas_msgs.msg import EncoderCount
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler


class OdometryNode(Node):
    def __init__(self):
        super().__init__("odometry")
        self.initialise_parameters()

        # Set up variables
        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_base = self.get_parameter("wheel_base").get_parameter_value().double_value
        self.counts_per_rev = self.get_parameter("counts_per_rev").get_parameter_value().integer_value
        self.prev_encoder_msg: EncoderCount | None = None

        # Odometry data set up
        self.x = 0.0  # position in xy plane
        self.y = 0.0
        self.th = 0.0
        self.dx = 0.0  # speeds in x/rotation
        self.dr = 0.0
        self.base_frame_id = self.declare_parameter('base_frame_id', 'base_link').value # robot's base frame
        self.odom_frame_id = self.declare_parameter('odom_frame_id', 'odom').value  # odometry reference frame
        self.lastDuinoOdomUpdateStamp = 0

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
                reliability=QoSReliabilityPolicy.RELIABLE
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

    def update_odom(self, msg: EncoderCount) -> None:
        """Update the pose data based on the encoder data"""
        # Calculate elapsed time
        current_time = msg.timestamp
        time_elapsed = (current_time - self.lastDuinoOdomUpdateStamp) / (1000*1000)

        L_counts = msg.left
        R_counts = msg.right

        # Convert counts/tick to m/tick
        Lw = L_counts * (360/48) * (1/74.8317)       #Counts/Tick * Angle/Count * Exact Gear Ratio = Angle/Tick * Exact Gear Ratio (Degrees/Tick)
        Lv = (Lw/360) * (math.pi*2*self.wheel_radius)      # meter/tick
        Rw = R_counts * (360/48) * (1/74.8317)       #Counts/Tick * Angle/Count * Exact Gear Ratio = Angle/Tick * Exact Gear Ratio (Degrees/Tick)
        Rv = (Rw/360) * (math.pi*2*self.wheel_radius)      # meter/tick

        # Displacement/Tick
        #Polar coords
        d_perTick = (Rv + Lv) / 2     #meter/Tick
        th_perTick = (Rv - Lv) / self.wheel_base      #Degrees/Tick
        #Cartesian coords
        if Lv != 0:
            x_perTick = math.cos(th_perTick) * d_perTick
            y_perTick = -math.sin(th_perTick) * d_perTick
            self.x = self.x + (math.cos(self.th) * x_perTick - math.sin(self.th)*y_perTick)
            self.y = self.y + (math.sin(self.th) * x_perTick + math.cos(self.th)*y_perTick)
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
        quaternion.z = math.sin(self.th/2)
        quaternion.w = math.cos(self.th/2)

        # Velocities 
        self.dx = d_perTick / time_elapsed   #meter/second
        self.dr = th_perTick / time_elapsed  #degrees/second

        self.lastDuinoOdomUpdateStamp = current_time


    def encoder_callback(self, msg: EncoderCount) -> None:
        """Callback function for the encoder data. Updates the odometry data"""
        if self.prev_encoder_msg is None:
            self.prev_encoder_msg = msg
            return
        
        self.update_odom(msg)
        self.prev_encoder_msg = msg

    def get_odometry(self) -> Odometry:
        """Get the current odometry data"""
        msg: Odometry = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        # Set the position
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        # Set the orientation (quaternion)
        quaternion = quaternion_from_euler(0, 0, self.th)
        msg.pose.pose.orientation = Quaternion(*quaternion)

        # Set the velocity
        msg.twist.twist.linear.x = self.dx
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = self.dr

        return msg

    def odometry_callback(self) -> None:
        """Publish the odometry data"""
        # Publish the odometry message
        odom_msg = self.get_odometry()
        self.odometry_publisher.publish(odom_msg)

def main(args: dict = None):
    rclpy.init(args=args)
    
    odometry = OdometryNode()
    try:
        rclpy.spin(odometry, MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass
    except rclpy.exceptions.ExternalShutdownException:
        sys.exit(1)
    finally:
        odometry.cleanup()  # Ensure GPIO cleanup happens
        odometry.destroy_node()

    rclpy.shutdown()

