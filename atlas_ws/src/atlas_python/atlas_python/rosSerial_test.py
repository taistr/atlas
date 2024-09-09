import serial
import time
from time import sleep
from threading import Lock
import logging

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rcl_interfaces.msg import ParameterType, ParameterDescriptor
from ament_index_python.packages import get_package_share_directory

from atlas_msgs.srv import MotionRequest


# Serial Data class for structured data storage & parsing
class duinoCmdStruct:
    def __init__(self):
        self.status = 400
        self.cmd = ''
        self.arg1 = 0.0
        self.arg2 = 0.0
        self.arg3 = 0.0
        self.arg4 = 0.0

    def to_serial_format(self):
        return f"{self.status}:{self.cmd}:{self.arg1}:{self.arg2}:{self.arg3}:{self.arg4}"

    def from_serial_format(self, serial_data):
        try:
            parts = serial_data.strip().split(':')
            if len(parts) == 6:
                self.status = int(parts[0])
                self.cmd = parts[1]
                self.arg1 = float(parts[2])
                self.arg2 = float(parts[3])
                self.arg3 = float(parts[4])
                self.arg4 = float(parts[5])
        except ValueError:
            print(f"Error parsing serial data: {serial_data}")

# Class to handle serial communications
class SerialComms(Node):
    # Sets up serial port
    def __init__(self):
        super().__init__("serial_comms")
        self.initialise_parameters()

        self.logger = logging.getLogger(__name__)

        self.serial = serial.Serial(
            port=self.get_parameter("port_name").value, 
            baudrate=self.get_parameter("BAUD_RATE").value, 
            timeout=1
        )

        self.mutex_serial = Lock()

        self.cmd_string = ""

        self.serialRxStruct = duinoCmdStruct()
        self.serialTxStruct = duinoCmdStruct()

        # Flags
        self.MOTION_COMPLETE = False
        self.SENT_FLAG = False

        # Setting up service
        self.srv = self.create_service(MotionRequest, 'motion_request', self.send_command)

        self.get_logger().info("Serial Comms initialised")

    # Initialises parameters
    def initialise_parameters(self) -> None:
        """Declare parameters for the serial comms node"""
        self.declare_parameter(
            "BAUD_RATE",
            value=115200,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Baud rate of the serial line."
            )
        )
        self.declare_parameter(
            "port_name",
            value='/dev/ttyACM0',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Name of the serial port."
            )
        )

    # Handles UART communications
    def send_command(self, request, response):
        self.serialTxStruct.arg1 = request.distance
        self.serialTxStruct.arg2 = request.angle
        self.serialTxStruct.arg3 = 0.0
        self.serialTxStruct.arg4 = 0.0
        self.serialTxStruct.status = 100
        self.serialTxStruct.cmd = 'b'
        # Send data over serial
        self.mutex_serial.acquire()
        try:
            print("sending")
            send_str = self.serialTxStruct.to_serial_format() + "\r"
            self.serial.write(send_str.encode('utf-8'))
            send_time = round(time.time() * 1000)
            print(f"Sent: {send_str.strip()}")

            sleep(0.5)
            
            while self.serial.in_waiting <=0:
                current_time = round(time.time() * 1000)
                if current_time > send_time + 4000:
                    raise Exception("Serial Timed Out: Retrying")

            if self.serial.in_waiting > 0:
                incoming_data = self.serial.readline().decode('utf-8')
                print(f"Received raw data: {incoming_data.strip()}")
                self.serialRxStruct.from_serial_format(incoming_data)
                print('RCVD:', self.serialRxStruct.status, self.serialRxStruct.cmd, self.serialRxStruct.arg1, self.serialRxStruct.arg2, self.serialRxStruct.arg3, self.serialRxStruct.arg4)

            if self.serialRxStruct.status == 200:
                return response
        except Exception as e:
            print(f"An error occured: {e}")
        finally:
            self.mutex_serial.release()

    def cleanup(self):
        # Close the serial port properly
        cmd_string = f"100:o:0:0:0:0"
        self.serial.write(cmd_string.encode("utf-8"))

        cmd_string = f"100:r:0:0:0:0"
        self.serial.write(cmd_string.encode("utf-8"))

        if self.serial.is_open:
            self.serial.close()


def main(args:dict = None):
    rclpy.init(args=args)
    serialCommsNode = SerialComms()

    try:
        rclpy.spin(serialCommsNode)
    except KeyboardInterrupt:
        pass
    serialCommsNode.destroy_node()
    try:
        rclpy.shutdown()
    except rclpy._rclpy_pybind11.RCLError:
        pass



if __name__ == '__main__':
    main()
