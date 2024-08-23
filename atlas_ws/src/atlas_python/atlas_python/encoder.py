import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from atlas_msgs.msg import EncoderCount
import time
import serial
from threading import Lock

class Encoder(Node):
    def __init__(self):
        super().__init__("encoder")
        self.initialise_parameters()
        self.encoder_count = EncoderCount(left=0, right=0, time_delta=0)

        self.serial = serial.Serial(
            port=self.get_parameter("port").value,
            baudrate=115200,
            timeout=0
        )

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
        serial_check_period = 1.0 / self.get_parameter("serial_rate").value
        self.encoder_publish_timer = self.create_timer(serial_check_period, self.timer_callback)

        self.mutex = Lock()

        self.get_logger().info("Encoder Node Online!")

    def initialise_parameters(self) -> None:
        """Declare parameters for the encoder node"""
        self.declare_parameter(
            "serial_rate",
            value=200,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Rate at which encoder data is published (Hz)",
            ),
        )
        self.declare_parameter(
            "port",
            value="/dev/ttyACM0",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Serial port for the encoder",
            ),
        )


    def readEncoder_cmd(self):
        #Send command to read encoder
        resp = self.send_command(f"e")
        self.get_logger().info(resp)
        if resp:
            return resp
        return []

    def timer_callback(self):
        try:
            # Read from the serial port
            response = self.readEncoder_cmd()
            if (response):
                line = response.decode('utf-8').strip()
                if line.startswith("L:") and ",R:" in line and ",Timestamp:" in line:
                    
                    # Parse the serial data
                    left_count = int(line.split(",R:")[0][2:])
                    right_count = int(line.split(",Timestamp:")[0].split(",R:")[1])
                    time_delta = int(line.split(",Timestamp:")[1])

                    
                    # Populate the ROS message
                    self.encoder_count.left = left_count
                    self.encoder_count.right = right_count
                    self.encoder_count.time_delta = time_delta

                    # Publish the message
                    self.encoder_publisher.publish(self.encoder_count)
                else:
                    self.get_logger().info(line)
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


    
    def cleanup(self):
        # Close the serial port properly
        if self.serial.is_open:
            self.serial.close()


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
