import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import RPi.GPIO as GPIO
except RuntimeError:  # RPi.GPIO throws errors when not on RPi
    from unittest.mock import MagicMock
    GPIO = MagicMock()

ENCODER_LEFT_A = 27
ENCODER_LEFT_B = 22
ENCODER_RIGHT_A = 23
ENCODER_RIGHT_B = 24

class EncoderTest(Node):
    def __init__(self):
        super().__init__("encoder_Test")
        self.get_logger().info("Encoder Test Initialising")

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(ENCODER_LEFT_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(ENCODER_LEFT_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(ENCODER_RIGHT_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(ENCODER_RIGHT_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        GPIO.add_event_detect(ENCODER_LEFT_A, GPIO.BOTH, callback=self._test_left)
        GPIO.add_event_detect(ENCODER_RIGHT_A, GPIO.BOTH, callback=self._test_right)

        self.create_timer(3, self.timer_callback_test)

    def _test_left(self, msg):
        self.get_logger().info("Left encoder event")
    
    def _test_right(self, msg):
        self.get_logger().info("Right encoder event")

    def timer_callback_test(self):
        self.get_logger().info("callback worked")
    
    def cleanup(self):
        GPIO.cleanup()  # Ensure the GPIO pins are reset

def main(args: dict = None):
    rclpy.init(args=args)
    
    encoder_test = EncoderTest()
    try:
        encoder_test.get_logger().info("Starting Actuation Test!")
        rclpy.spin(encoder_test)
    except KeyboardInterrupt:
        pass
    except rclpy.exceptions.ExternalShutdownException:
        sys.exit(1)
    finally:
        encoder_test.cleanup()  # Ensure GPIO cleanup happens
        encoder_test.destroy_node()

    rclpy.shutdown()

