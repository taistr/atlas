import RPi.GPIO as GPIO
import time

# Set up GPIO mode
GPIO.setmode(GPIO.BCM)

# Define pins for motor A
IN1 = 14
IN2 = 15
ENA = 18  # PWM pin for motor A

# Define pins for motor B
IN3 = 27
IN4 = 22
ENB = 17  # PWM pin for motor B

# Set up pins for motor A
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

# Set up pins for motor B
GPIO.setup(ENB, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

# Set up PWM for motor A
pwm_a = GPIO.PWM(ENA, 100)  # PWM frequency set to 100 Hz
pwm_a.start(0)  # Start PWM with 0% duty cycle

# Set up PWM for motor B
pwm_b = GPIO.PWM(ENB, 100)  # PWM frequency set to 100 Hz
pwm_b.start(0)  # Start PWM with 0% duty cycle

def set_motor_speed(pwm, speed):
    pwm.ChangeDutyCycle(speed)

def set_motor_direction(in1, in2, forward):
    if forward:
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
    else:
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)

try:
    while True:
        # Set motor A direction to forward
        set_motor_direction(IN1, IN2, True)
        # Set motor B direction to forward
        set_motor_direction(IN3, IN4, True)
        
        # Gradually increase the speed of both motors
        for speed in range(0, 101, 5):
            set_motor_speed(pwm_a, speed)
            set_motor_speed(pwm_b, speed)
            time.sleep(0.1)
        
        # Hold max speed for 2 seconds
        time.sleep(2)
        
        # Gradually decrease the speed of both motors
        for speed in range(100, -1, -5):
            set_motor_speed(pwm_a, speed)
            set_motor_speed(pwm_b, speed)
            time.sleep(0.1)
        
        # Stop for 2 seconds
        time.sleep(2)
        
        # Set motor A direction to backward
        set_motor_direction(IN1, IN2, False)
        # Set motor B direction to backward
        set_motor_direction(IN3, IN4, False)
        
        # Gradually increase the speed of both motors in reverse
        for speed in range(0, 101, 5):
            set_motor_speed(pwm_a, speed)
            set_motor_speed(pwm_b, speed)
            time.sleep(0.1)
        
        # Hold max speed for 2 seconds
        time.sleep(2)
        
        # Gradually decrease the speed of both motors
        for speed in range(100, -1, -5):
            set_motor_speed(pwm_a, speed)
            set_motor_speed(pwm_b, speed)
            time.sleep(0.1)
        
        # Stop for 2 seconds
        time.sleep(2)
except KeyboardInterrupt:
    pass

# Cleanup
pwm_a.stop()
pwm_b.stop()
GPIO.cleanup()
