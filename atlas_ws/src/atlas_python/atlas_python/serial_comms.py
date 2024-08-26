import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
import time
import serial
from std_msgs.msg import String  # or a custom message type
from threading import Lock

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.initialise_parameters()
        
        # Publisher
        self.serial_publisher = self.create_publisher(
            String, 
            'duino_serial_resp', 
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
            'duino_serial_cmd',
            self.serialSend_callback,
            qos_profile=QoSProfile(
                depth=10,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE
            )
        )

        # Serial
        self.serial = serial.Serial(
            port=self.get_parameter("port").value, 
            baudrate=115200, 
            timeout=1
        )
        
        self.mutex = Lock()

        #serial_publisher_period = 1.0 / self.get_parameter("serial_rate").value
        #self.serial_publish_timer = self.create_timer(serial_publisher_period, self.serialPublisher_callback)

        self.get_logger().info("Serial Node Online!")

    def initialise_parameters(self) -> None:
        """Declare parameters for the serial node"""
        self.declare_parameter(
            "serial_rate",
            value=200,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Rate at which serial data is published (Hz)",
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

    def serialSend_callback(self, msg: String):
        resp = self.send_command(msg.data)
        self.get_logger().info(resp)
        if resp:
            self.serialPublisher_callback(resp)
        return []

    def serialPublisher_callback(self, response):
        # if self.serial_port.in_waiting:
        #     line = self.serial_port.readline().decode('utf-8').rstrip()
        #     # Process line and publish
        #     msg = String()
        #     msg.data = line  # or process as needed
        #     self.publisher_.publish(msg)

        try:
            # Read from the serial port
            if (response):
                line = response.strip()
                # Publish the message
                self.serial_publisher.publish(line)
            else:
                self.get_logger().info(line)
        except Exception as e:
            self.get_logger().error(f"Error reading serial data: {e}")


    def send_command(self, cmd_string):
        # Send data over serial
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
    
    serialComms = SerialNode()
    try:
        rclpy.spin(serialComms, MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass
    except rclpy.exceptions.ExternalShutdownException:
        sys.exit(1)
    finally:
        serialComms.cleanup()  # Ensure GPIO cleanup happens
        serialComms.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()