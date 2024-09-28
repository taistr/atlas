import logging
import serial
from enum import Enum, auto
import time

class CommandState(Enum):
    IDLE = auto()
    SENDING = auto()
    WAITING_FOR_ACCEPTANCE = auto()
    WAITING_FOR_COMPLETION = auto()
    COMPLETED = auto()

class CommandStatusCode(Enum):
    CONTINUE = 100
    ACCEPTED = 202
    COMPLETED = 200
    FAILED = 400

class Command:
    def __init__(self, status=400, cmd='', arg1=0.0, arg2=0.0, arg3=0.0, arg4=0.0):
        self._logger = logging.getLogger(self.__class__.__name__)

        self.status = status
        self.cmd = cmd
        self.arg1 = arg1
        self.arg2 = arg2
        self.arg3 = arg3
        self.arg4 = arg4

    @property
    def serial_format(self) -> str:
        """
        Return the serial format of the command
        
        :return str: the formatted command to be sent over serial
        """
        return f"{self.status}:{self.cmd}:{self.arg1}:{self.arg2}:{self.arg3}:{self.arg4}"

    def from_serial_format(self, encoded_serial_data: str) -> None:
        """
        Populate the command object from a utf-8 encoded string from the Arduino.

        :param encoded_serial_data: the utf-8 encoded string from the Arduino
        """
        try:
            parts = encoded_serial_data.strip().split(':')
            if len(parts) == 6:
                self.status = int(parts[0])
                self.cmd = parts[1]
                self.arg1 = float(parts[2])
                self.arg2 = float(parts[3])
                self.arg3 = float(parts[4])
                self.arg4 = float(parts[5])
        except ValueError:
            self._logger.error(f"Error parsing serial data: {encoded_serial_data}")

    def reset(self):
        """
        Reset the command object to its default values
        """
        self.status = 400
        self.cmd = ""
        self.arg1 = 0.0
        self.arg2 = 0.0
        self.arg3 = 0.0
        self.arg4 = 0.0

SERIAL_INITIALISE_TIMEOUT = 1
DEFAULT_SERIAL_PORT = "/dev/ttyACM0"
DEFAULT_BAUD_RATE = 115200
ACCEPTANCE_TIMEOUT_NS = 4 * 1e9
COMPLETION_TIMEOUT_NS = 30 * 1e9
MOTOR_CMD = "m"
ACCEPTED_STATUS_CODE = 202
COMPLETED_STATUS_CODE = 200
FAILED_STATUS_CODE = 400

class SerialComms:
    logger: logging.Logger
    serial_port: str
    baud_rate: int

    def __init__(self, serial_port: str = DEFAULT_SERIAL_PORT, baud_rate: int = DEFAULT_BAUD_RATE) -> None:
        self.logger = logging.getLogger(self.__class__.__name__)

        self.serial_port = serial_port
        self.baud_rate = baud_rate

        self.serial_client = serial.Serial(
            port=self.serial_port, 
            baudrate=self.baud_rate, 
            timeout=SERIAL_INITIALISE_TIMEOUT,
        )

        self.command_string = ""
        self.command_state = CommandState.IDLE

        self.logger.info("SerialComms initialised")

    def transition_state(self, new_state: CommandState) -> None:
        """
        Transition the state of the serial communications object
        
        :param new_state: the new state to transition to
        """
        self.command_state = new_state
        self.logger.info(f"Transitioned to state: {self.command_state}")

    def start_motion(self, heading: float, distance: float) -> None:
        """
        Start the motion of the robot
        
        :param heading: the heading to move in
        :param distance: the distance to move
        """
        command = Command(
            status=CommandStatusCode.CONTINUE.value,
            cmd=MOTOR_CMD,
            arg1=float(f"{distance:.3f}"),
            arg2=float(f"{heading:.3f}"),
        )
        self.send_command(command)

    def send_command(self, command: Command) -> None:
        """
        Sends a command
        
        :param command: the command to send
        """

        # send data over serial
        self.transition_state(CommandState.SENDING)
        send_string = command.serial_format + "\r"
        try:
            self.serial_client.write(send_string.encode("utf-8"))
            send_time = time.perf_counter_ns()
        except serial.SerialException as e:
            self.logger.error(f"Error sending command: {e}")
            return
        except UnicodeEncodeError as e:
            self.logger.error(f"Error encoding command: {e}")
            return

        # wait 0.5s
        self.transition_state(CommandState.WAITING_FOR_ACCEPTANCE)
        time.sleep(0.5)

        received_command = Command()

        # if motor command then wait for acceptance message
        while self.command_state == CommandState.WAITING_FOR_ACCEPTANCE and command.cmd == MOTOR_CMD:
            if self.serial_client.in_waiting <= 0 and time.perf_counter_ns() - send_time > ACCEPTANCE_TIMEOUT_NS:
                raise TimeoutError("Timeout waiting for acceptance")
            elif self.serial_client.in_waiting > 0:
                serial_data = self.serial_client.readline().decode("utf-8")
                self.logger.info(f"Received raw data: {serial_data.strip()}")
                received_command.from_serial_format(serial_data)
                self.logger.info(
                    'RCVD: status=%s, cmd=%s, arg1=%s, arg2=%s, arg3=%s, arg4=%s',
                    received_command.status, received_command.cmd, received_command.arg1,
                    received_command.arg2, received_command.arg3, received_command.arg4
                )
                acceptance_time = time.perf_counter_ns()

            if received_command.status == ACCEPTED_STATUS_CODE:
                self.transition_state(CommandState.WAITING_FOR_COMPLETION)
            elif not received_command.status == FAILED_STATUS_CODE:
                raise ValueError(f"Unexpected status code: {received_command.status}")

        # wait for success message
        received_command.reset()
        while self.command_state == CommandState.WAITING_FOR_COMPLETION:
            if self.serial_client.in_waiting <= 0 and time.perf_counter_ns() - acceptance_time > COMPLETION_TIMEOUT_NS:
                raise TimeoutError("Timeout waiting for completion")
            elif self.serial_client.in_waiting > 0:
                serial_data = self.serial_client.readline().decode("utf-8")
                received_command.from_serial_format(serial_data)
            
            if received_command.status == COMPLETED_STATUS_CODE:
                self.transition_state(CommandState.COMPLETED)
            elif not received_command.status == FAILED_STATUS_CODE:
                raise ValueError(f"Unexpected status code: {received_command.status}")
            
        self.transition_state(CommandState.IDLE)




