import lgpio
from time import sleep
import logging
from enum import Enum

IN1 = 17  # Motor A input 1
IN2 = 27  # Motor A input 2
IN3 = 22  # Motor B input 1
IN4 = 23  # Motor B input 2
ENA = 12  # Motor A enable (PWM)
ENB = 13  # Motor B enable (PWM
PWM_FREQUENCY = 1000  # Hz
INITIAL_GPIO_LEVEL = 0  # Start the all GPIO pins at 0
INIITAL_PWM_DUTY_CYCLE = 0  # Start the PWM duty cycle at 0

class HoistDirection(Enum):
    FORWARD = 0
    BACKWARD = 1
    STOP = 2

class HoistMotor(Enum):
    MOTOR_A = 0
    MOTOR_B = 1

class Hoist:
    def __init__(self):
        """
        Initializes the Hoist object with the specified parameters.
        """
        self.logger = logging.getLogger(self.__class__.__name__)
        self.gpio_pins = [IN1, IN2, IN3, IN4, ENA, ENB]

        # Initialize the gpio chip
        self.gpio_handle = lgpio.gpiochip_open(0)
        if self.gpio_handle < 0:
            raise RuntimeError("Error: Could not open GPIO chip.")

        # Claim the GPIO pins for the motor inputs
        for pin in self.gpio_pins:
            result_code = lgpio.gpio_claim_output(self.gpio_handle, pin, INITIAL_GPIO_LEVEL)
            if result_code < 0:
                raise RuntimeError(f"Error: Could not claim GPIO pin {pin}.")
            
        # Set the PWM frequency and duty cycle for the motor enable pins
        lgpio.tx_pwm(self.gpio_handle, ENA, PWM_FREQUENCY, INIITAL_PWM_DUTY_CYCLE)
        lgpio.tx_pwm(self.gpio_handle, ENB, PWM_FREQUENCY, INIITAL_PWM_DUTY_CYCLE)
        

        self.logger.info(f"Hoist initialized with GPIO pins 
                         ENA: {ENA}, ENB: {ENB}, IN1: {IN1}, IN2: {IN2}, IN3: {IN3}, IN4: {IN4}")
        
    def __del__(self):
        """
        Releases the hoist resources upon object deletion.
        """
        lgpio.gpiochip_close(self.gpio_handle)
        self.logger.info("Hoist released")

    def control_motor(self, direction: HoistDirection, motor: HoistMotor, speed: int):
        """
        Control the specified motor in the specified direction at the specified speed.

        :param direction: The direction to move the motor.
        :param motor: The motor to control.
        :param speed: The speed at which to move the motor.
        """
        if speed < 0 or speed > 100:
            raise ValueError("Error: Speed must be between 0 and 100.")

        if motor == HoistMotor.MOTOR_A:
            if direction == HoistDirection.FORWARD:
                lgpio.gpio_write(self.gpio_handle, IN1, 1)
                lgpio.gpio_write(self.gpio_handle, IN2, 0)
            elif direction == HoistDirection.BACKWARD:
                lgpio.gpio_write(self.gpio_handle, IN1, 0)
                lgpio.gpio_write(self.gpio_handle, IN2, 1)
            elif direction == HoistDirection.STOP:
                lgpio.gpio_write(self.gpio_handle, IN1, 0)
                lgpio.gpio_write(self.gpio_handle, IN2, 0)

            lgpio.tx_pwm(self.gpio_handle, ENA, PWM_FREQUENCY, speed)

        elif motor == HoistMotor.MOTOR_B:
            if direction == HoistDirection.FORWARD:
                lgpio.gpio_write(self.gpio_handle, IN3, 1)
                lgpio.gpio_write(self.gpio_handle, IN4, 0)
            elif direction == HoistDirection.BACKWARD:
                lgpio.gpio_write(self.gpio_handle, IN3, 0)
                lgpio.gpio_write(self.gpio_handle, IN4, 1)
            elif direction == HoistDirection.STOP:
                lgpio.gpio_write(self.gpio_handle, IN3, 0)
                lgpio.gpio_write(self.gpio_handle, IN4, 0)

            lgpio.tx_pwm(self.gpio_handle, ENB, PWM_FREQUENCY, speed)

    def deposit_balls(self):
        """
        Deposit balls into the box.
        """
        # TODO: Implement the deposit balls method
        raise NotImplementedError("Method not implemented.")