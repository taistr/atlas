import sys
import time
import serial
from threading import Lock

#UART
UART_PORT = "/dev/ttyS0"
BAUD_RATE = 115200


class SerialComms():
    # Sets up serial port
    def __init__(self):
        self.serial = serial.Serial(
        port=UART_PORT, 
        baudrate=BAUD_RATE, 
        timeout=1
        )

        self.mutex_serial = Lock()

        self.cmd_string = ""
        
    # Handles sending and returning of serial commands
    def serialSend(self, cmd_string):
        resp = self.send_command(cmd_string)
        #self.get_logger().info(resp)
        if resp:
            return resp

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
                    print("Error: Serial timeout on command: " + cmd_string)
                    return ''
                value += c
            value = value.strip('\r')
            return value
        finally:
            self.mutex_serial.release()

    # def serialPublisher(self, response):
    #     # if self.serial_port.in_waiting:
    #     #     line = self.serial_port.readline().decode('utf-8').rstrip()
    #     #     # Process line and publish
    #     #     msg = String()
    #     #     msg.data = line  # or process as needed
    #     #     self.publisher_.publish(msg)

    #     try:
    #         # Read from the serial port
    #         if (response):
    #             line = response.strip()
    #             msg = String()
    #             msg.data = (line)
    #             # Publish the message
    #             self.serial_publisher.publish(msg)

    #             # Send Serial free msg on "duino_serial_resp"
    #             msg = String()
    #             msg.data = ("SERIAL:FREE")
    #             self.serial_publisher.publish(msg)
    #         else:
    #             raise Exception("No serial command received in duino_serial_cmd topic.")
    #     except Exception as e:
    #         # Send Serial free msg on "duino_serial_resp"
    #         msg = String()
    #         msg.data = ("SERIAL:FREE")
    #         self.serial_publisher.publish(msg)
    #         self.get_logger().error(f"Error reading serial data: {e}")


    def cleanup(self):
        # Close the serial port properly
        if self.serial.is_open:
            self.serial.close()


def serial_comms(command):
    SerialNode = SerialComms()

    # Initiates serial comms process
    SerialNode.serialSend(command)
