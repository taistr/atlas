import sys
from simple_pid.pid import PID

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from atlas_msgs.msg import EncoderCount

try:
    import RPi.GPIO as GPIO
except RuntimeError:  # RPi.GPIO throws errors when not on RPi
    from unittest.mock import MagicMock
    GPIO = MagicMock()

class MotorDriver(Node):
    def __init__(self):
        super().__init__("motor_driver")
        self.initialise_parameters()

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

        self.pid_left = PID(kp, ki, kd, setpoint=0)
        self.pid_right = PID(kp, ki, kd, setpoint=0)

        self.pid_left.output_limits = (0, 100) # Set output limits (assuming PWM range of 0-100)
        self.pid_right.output_limits = (0, 100)
        
        # Create ROS interfaces
        self.create_timer(
            1.0 / self.get_parameter("control_loop_rate").get_parameter_value().double_value,
        )
        self.create_subscription(
            EncoderCount,
            "atlas/encoder_count",
            self.encoder_callback,
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
    
    def encoder_callback(self, msg: EncoderCount) -> None:
        """Callback function for the encoder data. Updates the PID controllers"""


    def control_loop(self) -> None:
        pass

    def test_callback(self) -> None:
        self.get_logger().info("Test Callback!")
        if self.flag:
            self.set_motor_direction(IN1, IN2, True)
            self.set_motor_direction(IN3, IN4, True)
            self.set_motor_speed(self.pwm_a, 100)
            self.set_motor_speed(self.pwm_b, 100)
        else:
            self.set_motor_direction(IN1, IN2, False)
            self.set_motor_direction(IN3, IN4, False)
            self.set_motor_speed(self.pwm_a, 75)
            self.set_motor_speed(self.pwm_b, 75)

        self.flag = not self.flag


    # @staticmethod
    # def set_motor_speed(pwm: GPIO.PWM, speed: float):
    #     pwm.ChangeDutyCycle(speed)

    # def set_motor_direction(self, in1: int, in2: int, forward: bool):
    #     self.get_logger().info("forward" if forward else "backward")
    #     if forward:
    #         GPIO.output(in1, GPIO.HIGH)
    #         GPIO.output(in2, GPIO.LOW)
    #     else:
    #         GPIO.output(in1, GPIO.LOW)
    #         GPIO.output(in2, GPIO.HIGH)

    def cleanup(self):
        self.get_logger().info("Cleaning up GPIO and stopping PWM...")
        self.pwm_a.stop()
        self.pwm_b.stop()
        GPIO.cleanup()

def main(args: dict = None):
    rclpy.init(args=args)
    
    actuation = ActuationTest()
    try:
        actuation.get_logger().info("Starting Actuation Test!")
        rclpy.spin(actuation)
    except KeyboardInterrupt:
        pass
    except rclpy.exceptions.ExternalShutdownException:
        sys.exit(1)
    finally:
        actuation.cleanup()  # Ensure GPIO cleanup happens
        actuation.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
