import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from atlas_msgs.msg import EncoderCount
import serial.tools.list_ports
import serial

ARDUINO_SEND_PERIOD = 10000 #microseconds

class Encoder(Node):
    def __init__(self):
        super().__init__("encoder")
        self.initialise_parameters()
        self.encoder_count = EncoderCount(left=0, right=0, time_delta=0)

        self.serial = serial.Serial(
            port=self.find_arduino_port(0x2341, 0x0043),
            baudrate=115200,
            timeout=0
        )
        self.serial.write(b'.') # Send a byte to the Arduino to reset the encoder count

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

    def timer_callback(self):
        try:
            # Read from the serial port
            if self.serial.in_waiting > 0:
                line = self.serial.readline().decode('utf-8').strip()
                if line.startswith("L:") and ",R:" in line:
                    # Parse the serial data
                    left_count = int(line.split(",R:")[0][2:])
                    right_count = int(line.split(",R:")[1])

                    # Populate the ROS message
                    self.encoder_count.left = left_count
                    self.encoder_count.right = right_count
                    self.encoder_count.time_delta = ARDUINO_SEND_PERIOD

                    # Publish the message
                    self.encoder_publisher.publish(self.encoder_count)

        except Exception as e:
            self.get_logger().error(f"Error reading serial data: {e}")

    @staticmethod
    def find_arduino_port(vid, pid):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.vid == vid and port.pid == pid:
                return port.device
        return None

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
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        encoder.cleanup()  # Ensure GPIO cleanup happens
        encoder.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
