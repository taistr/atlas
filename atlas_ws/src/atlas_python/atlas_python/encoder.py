"""
This script is a ROS 2 node responsible for:
1. Sending read encoder commands to the Arduino Uno hardware interface, over serial
2. Capturing LEFT and RIGHT encoder counts & Arduino timestamp from serial
3. Publishing raw encoder counts & Arduino timestamp
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from atlas_msgs.msg import EncoderCount
from std_msgs.msg import String  # or a custom message type
import time
import serial
from threading import Lock

class Encoder(Node):
    def __init__(self):
        super().__init__("encoder")
        self.initialise_parameters()
        self.encoder_count = EncoderCount(left=0, right=0, timestamp=0) # Count, count, microseconds since last reset

        # Set up timer to publish encoder data
        self.encoder_publisher = self.create_publisher(
            EncoderCount, 
            "atlas/encoder_count", 
            qos_profile=QoSProfile(
                depth=10,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE
            )
        )

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

        # Subscriber
        self.subscription = self.create_subscription(
            String,
            'duino_serial_resp',
            self.serialResp_callback,
            qos_profile=QoSProfile(
                depth=10,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE
            )
        )

        # Encoder check timer
        encoder_check_period = 1.0 / self.get_parameter("encoder_rate").value
        self.encoder_publish_timer = self.create_timer(encoder_check_period, self.encoderCheck_callback)

        self.get_logger().info("Encoder Node Online!")

    def initialise_parameters(self) -> None:
        """Declare parameters for the encoder node"""
        self.declare_parameter(
            "encoder_rate",
            value=100,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Rate at which encoder data is published (Hz)",
            ),
        )

    def encoderCheck_callback(self):
        #Send command to read encoder
        msg = String()
        msg.data = "e"
        self.serial_publisher.publish(msg)
        
    def serialResp_callback(self, response: String):
        try:
            # Parse the incoming serial string
            if (response):
                if response.startswith("OK") and "READ_ENCODERS" in response and "L" in response and "R" in response and "Timestamp" in response:
                    # Parse the serial data
                    responseArr = response.split(".")
                    left_count = int(responseArr[2].split(':')[1])
                    right_count = int(responseArr[3].split(':')[1])
                    timestamp = int(responseArr[4].split(':')[1])

                    # Populate the ROS message
                    self.encoder_count.left = left_count
                    self.encoder_count.right = right_count
                    self.encoder_count.timestamp = timestamp

                    # Publish the message
                    self.encoder_publisher.publish(self.encoder_count)
                else:
                    self.get_logger().info(response)
        except Exception as e:
            self.get_logger().error(f"Error reading serial data: {e}")

    
    def send_command(self, cmd_string):
        self.mutex.acquire()

        try:
            cmd_string += "\r"
            self.serial.write(cmd_string.encode("utf-8"))

            c = ''
            value = ''
            while c != "\r":
                c = self.serial.read(1).decode("utf-8")
                if (c == ''):
                    print("Error: Serial timeout on command: " + cmd_string)
                    return ''
                value += c
            
            value = value.strip('\r')

            return value
        finally:
            self.mutex.release()


    
    # def cleanup(self):
    #     # Close the serial port properly
    #     if self.serial.is_open:
    #         self.serial.close()


def main(args: dict = None):
    rclpy.init(args=args)
    
    encoder = Encoder()
    try:
        rclpy.spin(encoder, MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass
    except rclpy.exceptions.ExternalShutdownException:
        sys.exit(1)
    finally:
        encoder.cleanup()  # Ensure GPIO cleanup happens
        encoder.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
