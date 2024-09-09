import serial
import time
from time import sleep
from threading import Lock
import logging

SERIAL_DEVICE = '/dev/ttyACM0'
BAUD_RATE = 115200
SENT_FLAG = False

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


# Configure logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

# Class to handle serial communications
class SerialComms():
    # Sets up serial port
    def __init__(self, uart_port: str = SERIAL_DEVICE, baud_rate: int = BAUD_RATE):
        self.logger = logging.getLogger(__name__)

        self.serial = serial.Serial(
            port=uart_port, 
            baudrate=baud_rate, 
            timeout=1
        )

        self.mutex_serial = Lock()

        self.cmd_string = ""

        # Flags
        self.MOTION_COMPLETE = False

        self.logger.info("SerialComms initialised")

    # Handles UART communications
    def send_command(self, serialStruct):
        # Send data over serial
        self.mutex_serial.acquire()
        try:
            print("sending")
            send_str = serialStruct.to_serial_format() + "\r"
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
                serialStruct.from_serial_format(incoming_data)
                print('RCVD:', serialStruct.status, serialStruct.cmd, serialStruct.arg1, serialStruct.arg2, serialStruct.arg3, serialStruct.arg4)

            return serialStruct
        except Exception as e:
            print(f"An error occured: {e}")
        finally:
            self.mutex_serial.release()

    def cleanup(self):
        # Close the serial port properly
        cmd_string = f"100:o:0:0:0:0"
        self.serial.write(cmd_string.encode("utf-8"))

        cmd_string = f"r"
        self.serial.write(cmd_string.encode("utf-8"))

        if self.serial.is_open:
            self.serial.close()



if __name__ == '__main__':
    try:
        serialRxStruct = duinoCmdStruct()
        serialTxStruct = duinoCmdStruct()
        serialInterface = SerialComms()
        
        sleep(2)  # Allow some time for the Arduino to reset

        serialTxStruct.arg1 = 0.0
        serialTxStruct.arg2 = 0.0
        serialTxStruct.arg3 = 0.0
        serialTxStruct.arg4 = 0.0
        serialTxStruct.cmd = 'b'
        serialTxStruct.status = 100

        while True:
            if not SENT_FLAG:
                response = serialInterface.send_command(serialTxStruct)
                if response:
                    SENT_FLAG = True

    except KeyboardInterrupt:
        serialInterface.cleanup()
    except Exception as e:
        print(f"Error: {e}")
        serialInterface.cleanup()
