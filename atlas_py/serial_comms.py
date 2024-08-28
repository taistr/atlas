import serial
from threading import Lock
import logging

# Configure logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')


class SerialComms():
    # Sets up serial port
    def __init__(self, uart_port: str = "/dev/ttyACM0", baud_rate: int = 115200):
        self.logger = logging.getLogger(__name__)

        self.serial = serial.Serial(
            port=uart_port, 
            baudrate=baud_rate, 
            timeout=1
        )

        self.mutex_serial = Lock()

        self.cmd_string = ""

        self.logger.info("SerialComms initialised")

    # Handles UART communications
    def send_command(self, cmd_string):
        # Send data over serial
        self.mutex_serial.acquire()
        try:
            cmd_string += "\r"

            serial.write(cmd_string.encode("utf-8"))
            c = ''
            value = ''
            while c != "\r":
                c = serial.read(1).decode("utf-8")
                if (c == ''):
                    self.logger.info("Error_Serial timeout on command: " + cmd_string)
                    return ''
                value += c
            value = value.strip('\r')
            return value
        finally:
            self.mutex_serial.release()

    def start_motion(self, heading: float, distance: float):
        # Convert heading and distance to float
        float_heading = float(heading)
        float_distance = float(distance)

        # Form the command string
        cmd_string = f"m {float_distance} {float_heading}"
        self.send_command(cmd_string)

    def cleanup(self):
        # Close the serial port properly
        if self.serial.is_open:
            self.serial.close()
