import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import sys
from simple_pid.pid import PID
from dataclasses import dataclass
try:
    import RPi.GPIO as GPIO
except RuntimeError:  # RPi.GPIO throws errors when not on RPi
    from unittest.mock import MagicMock
    GPIO = MagicMock()

@dataclass
class WheelVelocity:
    left: float
    right: float

class MotorDriver(Node):
    def __init__(self):
        super().__init__("motor_driver")
        self.initialise_parameters()

        self.measured_velocity = WheelVelocity(left=0.0, right=0.0)
        self.requested_velocity = WheelVelocity(left=0.0, right=0.0)

        # Set up variables
        self.wheel_base = self.get_parameter("wheel_base").get_parameter_value().double_value
        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.motor_driver_frequency = self.get_parameter("motor_driver_frequency").get_parameter_value().integer_value

        # Set up GPIO
        self.IN1 = self.get_parameter("in_1").get_parameter_value().integer_value
        self.IN2 = self.get_parameter("in_2").get_parameter_value().integer_value
        self.IN3 = self.get_parameter("in_3").get_parameter_value().integer_value
        self.IN4 = self.get_parameter("in_4").get_parameter_value().integer_value
        self.ENA = self.get_parameter("en_a").get_parameter_value().integer_value
        self.ENB = self.get_parameter("en_b").get_parameter_value().integer_value

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.IN1, GPIO.OUT)
        GPIO.setup(self.IN2, GPIO.OUT)
        GPIO.setup(self.ENA, GPIO.OUT)
        GPIO.setup(self.IN3, GPIO.OUT)
        GPIO.setup(self.IN4, GPIO.OUT)
        GPIO.setup(self.ENB, GPIO.OUT)

        self.pwm_a = GPIO.PWM(self.ENA, self.motor_driver_frequency) 
        self.pwm_b = GPIO.PWM(self.ENB, self.motor_driver_frequency)  

        self.pwm_b.start(0)  # Start PWM with 0% duty cycle
        self.pwm_a.start(0)  # Start PWM with 0% duty cycle

        # Set up PID controllers
        kp = self.get_parameter("kp").get_parameter_value().double_value
        ki = self.get_parameter("ki").get_parameter_value().double_value
        kd = self.get_parameter("kd").get_parameter_value().double_value

        self.pid_left: PID = PID(kp, ki, kd, setpoint=0, sample_time=None)
        self.pid_right: PID = PID(kp, ki, kd, setpoint=0, sample_time=None)

        self.pid_left.output_limits = (0, 100) # Set output limits (assuming PWM range of 0-100)
        self.pid_right.output_limits = (0, 100)
        
        # Create ROS interfaces
        self.control_loop_timer = self.create_timer(
            1.0 / self.get_parameter("control_loop_rate").get_parameter_value().double_value,
            self.control_loop
        )
        self.odometry_subscriber = self.create_subscription(
            Odometry,
            "atlas/odometry",
            self.odometry_callback,
            qos_profile=QoSProfile(
                depth=10,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE
            )
        )
        self.velocity_request_subscriber = self.create_subscription(
            Twist,
            "atlas/velocity_request",
            self.velocity_request_callback,
            qos_profile=QoSProfile(
                depth=10,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE
            )
        )
        self.get_logger().info("Motor Driver Initialised!")


    def initialise_parameters(self) -> None:
        """Declare parameters for the motor driver node"""
        self.declare_parameter(
            "control_loop_rate",
            value=100,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Rate at which the control loop runs (Hz)"
            )
        )
        self.declare_parameter(
            "motor_driver_frequency",
            value=100,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="Frequency of the PWM signal for the motor driver (Hz)"
            )
        )
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
        self.declare_parameter(
            "kp",
            value=0.1,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Proportional gain for the PID controller"
            )
        )
        self.declare_parameter(
            "ki",
            value=0.01,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Integral gain for the PID controller"
            )
        )
        self.declare_parameter(
            "kd",
            value=0.05,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Derivative gain for the PID controller"
            )
        )
        self.declare_parameter(
            "in_1",
            value=9,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="GPIO pin for IN1"
            ),
        )
        self.declare_parameter(
            "in_2",
            value=11,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="GPIO pin for IN2"
            ),
        )
        self.declare_parameter(
            "in_3",
            value=15,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="GPIO pin for IN3"
            ),
        )
        self.declare_parameter(
            "in_4",
            value=14,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="GPIO pin for IN4"
            ),
        )
        self.declare_parameter(
            "en_a",
            value=13,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="GPIO pin for ENA"
            ),
        )
        self.declare_parameter(
            "en_b",
            value=12,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="GPIO pin for ENB"
            ),
        )
    
    def odometry_callback(self, msg: Odometry) -> None:
        """Callback function for the odometry data. Updates the measured velocity"""

        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z

        # Calculate the left and right wheel velocities
        self.measured_velocity.left = v - w * self.wheel_base / 2
        self.measured_velocity.right = v + w * self.wheel_base / 2



    def velocity_request_callback(self, msg: Twist) -> None:
        """Callback function for the velocity request. Updates the requested velocity"""
        v = msg.linear.x
        w = msg.angular.z

        # Calculate the left and right wheel velocities
        self.requested_velocity.left = v - w * self.wheel_base / 2
        self.requested_velocity.right = v + w * self.wheel_base / 2

    def control_loop(self) -> None:
        """Control loop for the motor driver. Updates the motor speeds based on the PID controllers"""
        dt = 1.0 / self.get_parameter("control_loop_rate").get_parameter_value().double_value

        self.pid_left.setpoint = self.requested_velocity.left
        self.pid_right.setpoint = self.requested_velocity.right

        left_output = self.pid_left(self.measured_velocity.left, dt=dt)
        right_output = self.pid_right(self.measured_velocity.right, dt=dt)

        self.set_motor_speed(left=True, speed=left_output)
        self.set_motor_speed(left=False, speed=right_output)

    def set_motor_speed(self, left: bool, speed: float):
        """Set motor speed and direction based on side (left/right) and speed."""
        if left:
            pwm = self.pwm_a
            in1, in2 = self.IN1, self.IN2
        else:
            pwm = self.pwm_b
            in1, in2 = self.IN3, self.IN4

        # Determine direction based on the sign of speed
        if speed >= 0:
            GPIO.output(in1, GPIO.HIGH)
            GPIO.output(in2, GPIO.LOW)
        else:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.HIGH)

        # Set the PWM duty cycle
        pwm.ChangeDutyCycle(abs(speed))

    def cleanup(self):
        self.get_logger().info("Cleaning up GPIO and stopping PWM...")
        self.pwm_a.stop()
        self.pwm_b.stop()
        GPIO.cleanup()

def main(args: dict = None):
    rclpy.init(args=args)
    
    motor_driver = MotorDriver()
    try:
        rclpy.spin(motor_driver, MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass
    except rclpy.exceptions.ExternalShutdownException:
        sys.exit(1)
    finally:
        motor_driver.cleanup()  # Ensure GPIO cleanup happens
        motor_driver.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
