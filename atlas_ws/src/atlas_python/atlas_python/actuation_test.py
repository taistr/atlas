import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
from atlas_msgs.srv import MotorCommand

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

        # Set up timer to request motor drive
        self.cli = self.create_client(MotorCommand, 'motor_command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = MotorCommand.Request()


        #self.test_timer = self.create_timer(3, self.test_callback)
        self.flag = False

    def send_request(self, heading, distance):
        self.req.heading = heading
        self.req.distance = distance
        return self.cli.call_async(self.req)

    #def cleanup(self):
        # Stops the motors
        # msg = String()
        # msg.data = "o 0 0"
        # self.serial_publisher.publish(msg)

def main(args: dict = None):
    rclpy.init(args=args)
    
    actuation = ActuationTest()

    future = actuation.send_request(float(sys.argv[2]), float(sys.argv[1]))
    rclpy.spin_until_future_complete(actuation, future)
    response = future.result()
    actuation.get_logger().info("Motor request sent")
    #actuation.cleanup()  # Ensure cleanup happens
    actuation.destroy_node()
    
    # try:
    #     actuation.get_logger().info("Starting Actuation Test!")
    #     rclpy.spin(actuation)
    # except KeyboardInterrupt:
    #     pass
    # except ExternalShutdownException:
    #     sys.exit(1)
    # finally:
    #     actuation.cleanup()  # Ensure cleanup happens
    #     actuation.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
