import RPi.GPIO as GPIO
from time import sleep
import logging
from enum import Enum

IN1 = 17  # Motor A input 1
IN2 = 27  # Motor A input 2
IN3 = 22  # Motor B input 1
IN4 = 23  # Motor B input 2
ENA = 12  # Motor A enable (PWM)
ENB = 13  # Motor B enable (PWM)
PWM_FREQUENCY = 1000  # Hz
INITIAL_GPIO_LEVEL = 0  # Start all GPIO pins at 0
INITIAL_PWM_DUTY_CYCLE = 0  # Start the PWM duty cycle at 0

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

        # Initialize the GPIO library
        GPIO.setmode(GPIO.BCM)  # Use BCM pin numbering
        GPIO.setwarnings(False)  # Disable GPIO warnings

        # Claim the GPIO pins for the motor inputs
        for pin in self.gpio_pins:
            GPIO.setup(pin, GPIO.OUT, initial=INITIAL_GPIO_LEVEL)
            
        # Set the PWM frequency and duty cycle for the motor enable pins
        self.pwm_A = GPIO.PWM(ENA, PWM_FREQUENCY)
        self.pwm_B = GPIO.PWM(ENB, PWM_FREQUENCY)
        
        # Start PWM with the initial duty cycle
        self.pwm_A.start(INITIAL_PWM_DUTY_CYCLE)
        self.pwm_B.start(INITIAL_PWM_DUTY_CYCLE)

        self.logger.info(f"Hoist initialized with GPIO pins "
                         f"ENA: {ENA}, ENB: {ENB}, IN1: {IN1}, IN2: {IN2}, IN3: {IN3}, IN4: {IN4}")
        
    def __del__(self):
        """
        Releases the hoist resources upon object deletion.
        """
        self.pwm_A.stop()
        self.pwm_B.stop()
        GPIO.cleanup()  # Clean up GPIO resources
        self.logger.info("Hoist released")

    def control_motor(self, direction: HoistDirection, motor: HoistMotor, speed: int):
        """
        Control the specified motor in the specified direction at the specified speed.

        :param direction: The direction to move the motor.
        :param motor: The motor to control.
        :param speed: The speed at which to move the motor (0-100).
        """
        if speed < 0 or speed > 100:
            raise ValueError("Error: Speed must be between 0 and 100.")

        if motor == HoistMotor.MOTOR_A:
            if direction == HoistDirection.FORWARD:
                GPIO.output(IN1, GPIO.HIGH)
                GPIO.output(IN2, GPIO.LOW)
            elif direction == HoistDirection.BACKWARD:
                GPIO.output(IN1, GPIO.LOW)
                GPIO.output(IN2, GPIO.HIGH)
            elif direction == HoistDirection.STOP:
                GPIO.output(IN1, GPIO.LOW)
                GPIO.output(IN2, GPIO.LOW)

            self.pwm_A.ChangeDutyCycle(speed)

        elif motor == HoistMotor.MOTOR_B:
            if direction == HoistDirection.FORWARD:
                GPIO.output(IN3, GPIO.HIGH)
                GPIO.output(IN4, GPIO.LOW)
            elif direction == HoistDirection.BACKWARD:
                GPIO.output(IN3, GPIO.LOW)
                GPIO.output(IN4, GPIO.HIGH)
            elif direction == HoistDirection.STOP:
                GPIO.output(IN3, GPIO.LOW)
                GPIO.output(IN4, GPIO.LOW)

            self.pwm_B.ChangeDutyCycle(speed)

    def deposit_balls(self):
        """
        Deposit balls into the box.
        """
        # TODO: Implement the deposit balls method
        raise NotImplementedError("Method not implemented.")
