# Generic Imports
import time
from threading import Lock, Thread, Event
import sys
from dataclasses import dataclass
import asyncio

# ROS2 Imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String  # or a custom message type
from atlas_msgs.srv import MotorCommand


class MotorDriver(Node):
    def __init__(self):
        super().__init__("motor_driver")
        self.initialise_parameters()

        # Async Set up
        self.future = asyncio.Future()

        # Service set up
        self.srv = self.create_service(
            MotorCommand, 
            'atlas/motor_command', 
            self.motorCommands_callback,
        )

        #Serial Subscriber
        self.serial_subscription = self.create_subscription(
            String,
            'duino_serial_resp',
            self.serialResp_callback,
            qos_profile=QoSProfile(
                depth=10,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                reliability=QoSReliabilityPolicy.RELIABLE
            )
        )

        # Serial Publisher
        self.serial_publisher = self.create_publisher(
            String, 
            "duino_serial_cmd", 
            qos_profile=QoSProfile(
                depth=10,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE
            )
        )

        # Motor Command blocker flag
        self.motor_command_complete = False

        self.get_logger().info("Motor Driver Initialised!")


    def initialise_parameters(self) -> None:
        """Declare parameters for the motor driver node - Arduino hardware"""

        # Robot params
        self.declare_parameter(
            "wheel_radius",
            value=0.03,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Radius of the wheel (m)"
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

        # PID - Straight params
        self.declare_parameter(
            "L_kp",
            value=0.1,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Proportional gain for the PID controller"
            )
        )
        self.declare_parameter(
            "L_ki",
            value=0.05,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Integral gain for the PID controller"
            )
        )
        self.declare_parameter(
            "L_kd",
            value=5,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Derivative gain for the PID controller"
            )
        )
        self.declare_parameter(
            "L_ko",
            value=10,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Overall gain for the PID controller"
            )
        )
        self.declare_parameter(
            "R_kp",
            value=0.1,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Proportional gain for the PID controller"
            )
        )
        self.declare_parameter(
            "R_ki",
            value=0.05,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Integral gain for the PID controller"
            )
        )
        self.declare_parameter(
            "R_kd",
            value=5,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Derivative gain for the PID controller"
            )
        )
        self.declare_parameter(
            "R_ko",
            value=10,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Overall gain for the PID controller"
            )
        )

        # PID - Turning params
        self.declare_parameter(
            "Lt_kp",
            value=0.1,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Proportional gain for the PID controller"
            )
        )
        self.declare_parameter(
            "Lt_ki",
            value=0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Integral gain for the PID controller"
            )
        )
        self.declare_parameter(
            "Lt_kd",
            value=5,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Derivative gain for the PID controller"
            )
        )
        self.declare_parameter(
            "Lt_ko",
            value=10,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Overall gain for the PID controller"
            )
        )
        self.declare_parameter(
            "Rt_kp",
            value=0.1,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Proportional gain for the PID controller"
            )
        )
        self.declare_parameter(
            "Rt_ki",
            value=0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Integral gain for the PID controller"
            )
        )
        self.declare_parameter(
            "Rt_kd",
            value=5,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Derivative gain for the PID controller"
            )
        )
        self.declare_parameter(
            "Rt_ko",
            value=10,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Overall gain for the PID controller"
            )
        )
    
    ####### Non-future client response ########
    # def serialResp_callback(self, msg: String):
    #     response = str(msg.data).strip()
    #     if (response):
    #         if "MOTION_CONTROLLER" in response and "Straight Complete" in response:
    #             self.motor_command_complete = True


    # def motorCommands_callback(self, request, response):
    #     msg = String()
    #     msg.data = "m " + str(request.distance) + " " + str(request.heading)
    #     self.serial_publisher.publish(msg)
    #     while self.motor_command_complete:
    #          return response
    ###################################################
        
    ########## Future Async client response #######
    def serialResp_callback(self, msg: String):
        response = str(msg.data).strip()
        if (response):
            if "MOTION_CONTROLLER" in response and "Straight Complete" in response:
                self.future.set_result("MOTION COMPLETE")

    
    async def motorCommands_callback(self, request, response):
        msg = String()
        msg.data = "m " + str(request.distance) + " " + str(request.heading)
        self.serial_publisher.publish(msg)
        publish_time = Node('motor_driver').get_clock().now()

        if self.future.result("MOTION COMPLETE"):
            return response
        elif publish_time == Node("motor_driver").get_clock().now():
            self.get_logger().info("Motor command response timed out.")
            return response

    def cleanup(self):
        msg = String()
        msg.data = "o 0 0"
        self.serial_publisher.publish(msg) 

def main(args: dict = None):
    rclpy.init(args=args)
    
    motor_driver = MotorDriver()
    try:
        rclpy.spin(motor_driver, MultiThreadedExecutor())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    motor_driver.cleanup()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
