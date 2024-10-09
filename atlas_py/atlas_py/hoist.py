import RPi.GPIO as GPIO
from time import sleep
import logging
from enum import Enum

# RPi.GPIO pin assignments (BCM numbering)
IN1 = 17
IN2 = 27
IN3 = 22
IN4 = 23
ENA = 12
ENB = 13

PWM_FREQUENCY = 1000

class HoistDirection(Enum):
    UP = 0
    DOWN = 1
    STOP = 2
    RELAX = 3

class HoistMotor(Enum):
    MOTOR_FRONT = 0
    MOTOR_REAR = 1

class Hoist:
    def __init__(self):
        self.logger = logging.getLogger(self.__class__.__name__)

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Front
        GPIO.setup(IN1, GPIO.OUT)
        GPIO.setup(IN2, GPIO.OUT)
        GPIO.setup(ENA, GPIO.OUT)

        # Rear
        GPIO.setup(IN3, GPIO.OUT)
        GPIO.setup(IN4, GPIO.OUT)
        GPIO.setup(ENB, GPIO.OUT)

        self.pwm_front = GPIO.PWM(ENA, 1000)  # FRONT MOTOR
        self.pwm_rear = GPIO.PWM(ENB, 1000)  # REAR MOTOR

        self.pwm_front.start(0)
        self.pwm_rear.start(0)

        self.logger.info("Hoist online!")

    def cleanup(self):
        self.pwm_front.stop()
        self.pwm_rear.stop()
        GPIO.cleanup()
        self.logger.info("Hoist cleanup!")

    def move(self, motor: HoistMotor, direction: HoistDirection, duty_cycle: int = 0):
        """
        Move the hoist in the specified direction with the specified duty cycle.
        Note - stopping the motor should use a duty cycle of 0.

        :param motor: The motor to move.
        
        """
        if motor == HoistMotor.MOTOR_REAR:
            match direction:
                case HoistDirection.UP:
                    GPIO.output(IN1, GPIO.HIGH)
                    GPIO.output(IN2, GPIO.LOW)
                    self.pwm_front.ChangeDutyCycle(duty_cycle)
                case HoistDirection.DOWN:
                    GPIO.output(IN1, GPIO.LOW)
                    GPIO.output(IN2, GPIO.HIGH)
                    self.pwm_front.ChangeDutyCycle(duty_cycle)
                case HoistDirection.STOP:
                    GPIO.output(IN1, GPIO.HIGH)
                    GPIO.output(IN2, GPIO.HIGH)
                    self.pwm_front.ChangeDutyCycle(0)
                case HoistDirection.RELAX:
                    GPIO.output(IN1, GPIO.HIGH)
                    GPIO.output(IN2, GPIO.HIGH)
                    self.pwm_front.ChangeDutyCycle(0)
        else:
            match direction:
                case HoistDirection.UP:
                    GPIO.output(IN3, GPIO.HIGH)
                    GPIO.output(IN4, GPIO.LOW)
                    self.pwm_rear.ChangeDutyCycle(duty_cycle)
                case HoistDirection.DOWN:
                    GPIO.output(IN3, GPIO.LOW)
                    GPIO.output(IN4, GPIO.HIGH)
                    self.pwm_rear.ChangeDutyCycle(duty_cycle)
                case HoistDirection.STOP:
                    GPIO.output(IN3, GPIO.HIGH)
                    GPIO.output(IN4, GPIO.HIGH)
                    self.pwm_rear.ChangeDutyCycle(0)
                case HoistDirection.RELAX:
                    GPIO.output(IN3, GPIO.HIGH)
                    GPIO.output(IN4, GPIO.HIGH)
                    self.pwm_front.ChangeDutyCycle(0)

    def deposit(self):
        """
        Deposit the ball into the basket.
        """

        self.move(HoistMotor.MOTOR_FRONT, HoistDirection.UP, 100)
        sleep(1)
        
        self.move(HoistMotor.MOTOR_REAR, HoistDirection.UP, 100)
        sleep(5)

        self.move(HoistMotor.MOTOR_FRONT, HoistDirection.STOP)
        sleep(3)

        self.move(HoistMotor.MOTOR_REAR, HoistDirection.STOP)
        sleep(5)

        self.move(HoistMotor.MOTOR_REAR, HoistDirection.DOWN, 100)
        self.move(HoistMotor.MOTOR_FRONT, HoistDirection.DOWN, 100)
        sleep(5)

        self.move(HoistMotor.MOTOR_FRONT, HoistDirection.RELAX)
        self.move(HoistMotor.MOTOR_REAR, HoistDirection.RELAX)