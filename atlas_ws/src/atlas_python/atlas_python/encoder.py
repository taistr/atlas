import sys
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
from atlas_msgs.msg import EncoderCount
import time

try:
    import RPi.GPIO as GPIO
except RuntimeError:  # RPi.GPIO throws errors when not on RPi
    from unittest.mock import MagicMock
    GPIO = MagicMock()


class Encoder(Node):
    def __init__(self):
        super().__init__("encoder")
        self.initialise_parameters()
        self.encoder_count = EncoderCount(left=0, right=0)

        # Get parameters
        self.ENCODER_LEFT_A = self.get_parameter("encoder_left_a").get_parameter_value().integer_value
        self.ENCODER_LEFT_B = self.get_parameter("encoder_left_b").get_parameter_value().integer_value  
        self.ENCODER_RIGHT_A = self.get_parameter("encoder_right_a").get_parameter_value().integer_value
        self.ENCODER_RIGHT_B = self.get_parameter("encoder_right_b").get_parameter_value().integer_value

        # Set-up GPIO pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.ENCODER_LEFT_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.ENCODER_LEFT_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.ENCODER_RIGHT_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.ENCODER_RIGHT_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.ENCODER_LEFT_A, GPIO.BOTH, callback=self.left_encoder_callback)
        GPIO.add_event_detect(self.ENCODER_RIGHT_A, GPIO.BOTH, callback=self.right_encoder_callback)

        # Set up timer to publish encoder data
        self.encoder_publisher = self.create_publisher(
            EncoderCount, 
            "atlas/encoder_count", 
            qos_profile=QoSProfile(
                depth=10,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.BEST_EFFORT
            )
        )
        publish_period = 1.0 / self.get_parameter("publish_rate").get_parameter_value().double_value
        self.encoder_publish_timer = self.create_timer(publish_period, self.timer_callback)

        self.get_logger().info("Encoder Node Online!")


    def initialise_parameters(self) -> None:
        """Declare parameters for the encoder node"""
        self.declare_parameter(
            "publish_rate",
            value=50,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Rate at which encoder data is published (Hz)",
            ),
        )
        self.declare_parameter(
            "encoder_left_a",
            value=27,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="GPIO pin for left encoder A",
            ),
        )
        self.declare_parameter(
            "encoder_left_b",
            value=22,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="GPIO pin for left encoder B",
            ),
        )
        self.declare_parameter(
            "encoder_right_a",
            value=23,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="GPIO pin for right encoder A",
            ),
        )
        self.declare_parameter(
            "encoder_right_b",
            value=24,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="GPIO pin for right encoder B",
            ),
        )

    def left_encoder_callback(self, channel):
        """Callback function for left encoder"""
        if GPIO.input(self.ENCODER_LEFT_A) == GPIO.input(self.ENCODER_LEFT_B):
            self.encoder_count.left += 1
        else:
            self.encoder_count.left -= 1
        
    def right_encoder_callback(self, channel):
        """Callback function for right encoder"""
        if GPIO.input(self.ENCODER_RIGHT_A) == GPIO.input(self.ENCODER_RIGHT_B):
            self.encoder_count.right += 1
        else:
            self.encoder_count.right -= 1

    def timer_callback(self):
        self.encoder_count.time = time.perf_counter_ns()
        self.encoder_publisher.publish(self.encoder_count)
    
    def cleanup(self):
        GPIO.cleanup()  # Ensure the GPIO pins are reset

def main(args: dict = None):
    rclpy.init(args=args)
    
    encoder = Encoder()
    try:
        encoder.get_logger().info("Starting Actuation Test!")
        rclpy.spin(encoder)
    except KeyboardInterrupt:
        pass
    except rclpy.exceptions.ExternalShutdownException:
        sys.exit(1)
    finally:
        encoder.cleanup()  # Ensure GPIO cleanup happens
        encoder.destroy_node()

    rclpy.shutdown()

