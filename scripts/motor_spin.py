import RPi.GPIO as GPIO
import time
from scripts.odometry import *

# Set up GPIO mode
GPIO.setmode(GPIO.BCM)

# Define Tick Rate for motor drives - Anh
TICK_RATE = 100 #100Hz

# Define pins for motor A
IN1 = 14
IN2 = 15
ENA = 18  # PWM pin for motor A

# Define pins for motor B
IN3 = 27
IN4 = 22
ENB = 17  # PWM pin for motor Bpy

# Set up pins for motor A
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

# Set up pins for motor B
GPIO.setup(ENB, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

# Set up PWM for motor A
pwm_a = GPIO.PWM(ENA, TICK_RATE)  # PWM frequency set to 100 Hz
pwm_a.start(0)  # Start PWM with 0% duty cycle

# Set up PWM for motor B
pwm_b = GPIO.PWM(ENB, TICK_RATE)  # PWM frequency set to 100 Hz
pwm_b.start(0)  # Start PWM with 0% duty cycle

def set_motor_speed(pwm, speed):
    pwm.ChangeDutyCycle(speed)

def set_motor_direction(in1, in2, direction):
    if direction == "forward":
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
    elif direction == "reverse":
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
    else:
        return

def motorDrive(motorID, speed, direction):
    """
    Drives a selected motor at a selected speed and direction. This is a wrapper for the functions that Tyson wrote.
        motorID (string): "A", "B"
        speed (integer): PWM Duty Cycle (0 -> 100)   -- As of 15/08/2024
        direction (string): "forward" or "reverse"
    """
    if motorID == "A":
        set_motor_direction(IN1, IN2, direction)
        set_motor_speed(pwm_a, speed)
    elif motorID == "B":
        set_motor_direction(IN3, IN4, direction)
        set_motor_speed(pwm_a, speed)
    else:
        return
    
def motorCleanUp():
    # Cleanup
    pwm_a.stop()
    pwm_b.stop()
    GPIO.cleanup()
    

# Testing Block------------------------------------------------
try:
    while True:
        # # Set motor A direction to forward
        # set_motor_direction(IN1, IN2, True)
        # # Set motor B direction to forward
        # set_motor_direction(IN3, IN4, True)
        
        # Gradually increase the speed of both motors
        for speed in range(0, 101, 5):
            motorDrive("A", speed, "forward")
            motorDrive("B", speed, "forward")
            time.sleep(0.1)
        
        # Hold max speed for 2 seconds
        time.sleep(2)
        
        # Gradually decrease the speed of both motors
        for speed in range(100, -1, -5):
            motorDrive("A", speed, "forward")
            motorDrive("B", speed, "forward")
            time.sleep(0.1)
        
        # Stop for 2 seconds
        time.sleep(2)
        
        # # Set motor A direction to backward
        # set_motor_direction(IN1, IN2, False)
        # # Set motor B direction to backward
        # set_motor_direction(IN3, IN4, False)
        
        # Gradually increase the speed of both motors in reverse
        for speed in range(0, 101, 5):
            motorDrive("A", speed, "reverse")
            motorDrive("B", speed, "reverse")
            time.sleep(0.1)
        
        # Hold max speed for 2 seconds
        time.sleep(2)
        
        # Gradually decrease the speed of both motors
        for speed in range(100, -1, -5):
            motorDrive("A", speed, "reverse")
            motorDrive("B", speed, "reverse")
            time.sleep(0.1)
        
        # Stop for 2 seconds
        time.sleep(2)
except KeyboardInterrupt:
    pass
#------------------------------------

motorCleanUp()

