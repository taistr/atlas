import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String

# try:
#     import RPi.GPIO as GPIO
# except RuntimeError:  # RPi.GPIO throws errors when not on RPi
#     from unittest.mock import MagicMock
#     GPIO = MagicMock()

# IN1 = 9
# IN2 = 11
# ENA = 13

# IN3 = 15
# IN4 = 14 # reversed
# ENB = 12

class ActuationTest(Node):
    def __init__(self):
        super().__init__("actuation_test")
        self.get_logger().info("Actuation Test Initialising!")

        # Set up timer to publish serial command
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

        self.test_timer = self.create_timer(3, self.test_callback)
        self.flag = False



    def test_callback(self) -> None:
        self.get_logger().info("Test Callback!")
        # if self.flag:
        #     self.set_motor_direction(IN1, IN2, True)
        #     self.set_motor_direction(IN3, IN4, True)
        #     self.set_motor_speed(self.pwm_a, 100)
        #     self.set_motor_speed(self.pwm_b, 100)
        # else:
        #     self.set_motor_direction(IN1, IN2, False)
        #     self.set_motor_direction(IN3, IN4, False)
        #     self.set_motor_speed(self.pwm_a, 75)
        #     self.set_motor_speed(self.pwm_b, 75)

        msg = String()
        msg.data = "o 255 255"
        self.serial_publisher.publish(msg)

    # def cleanup(self):
    #     # Close the serial port properly
    #     if self.serial.is_open:
    #         self.serial.close()

def main(args: dict = None):
    rclpy.init(args=args)
    
    actuation = ActuationTest()
    try:
        actuation.get_logger().info("Starting Actuation Test!")
        rclpy.spin(actuation)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        actuation.cleanup()  # Ensure GPIO cleanup happens
        actuation.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
