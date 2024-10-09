import RPi.GPIO as GPIO
from time import sleep

# RPi.GPIO pin assignments (BCM numbering)
IN1 = 17
IN2 = 27
IN3 = 22
IN4 = 23
ENA = 12
ENB = 13

# Initialize RPi.GPIO
GPIO.setmode(GPIO.BCM)  # Use BCM pin numbering
GPIO.setwarnings(False)  # Disable GPIO warnings

# Set pin modes
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

# Set up PWM
pwm_A = GPIO.PWM(ENA, 1000)  # 1kHz frequency
pwm_B = GPIO.PWM(ENB, 1000)  # 1kHz frequency

# Start PWM with 0% duty cycle (motors off)
pwm_A.start(0)
pwm_B.start(0)

print("moving motors")

#! Raise the hoist up to ramp 
GPIO.output(IN1, GPIO.HIGH)  # Set IN1 to HIGH
GPIO.output(IN2, GPIO.LOW)   # Set IN2 to LOW
GPIO.output(IN3, GPIO.HIGH)  # Set IN1 to HIGH
GPIO.output(IN4, GPIO.LOW)   # Set IN2 to LOW
pwm_A.ChangeDutyCycle(50)    # 50% duty cycle
pwm_B.ChangeDutyCycle(50)

sleep(4.5)

#! Stop the front motor and back motor

GPIO.output(IN1, GPIO.HIGH)
GPIO.output(IN2, GPIO.HIGH)
GPIO.output(IN3, GPIO.HIGH)  # Set IN1 to HIGH
GPIO.output(IN4, GPIO.HIGH)   # Set IN2 to LOW
pwm_A.ChangeDutyCycle(0)  # Set PWM to 0% (stop motor)
pwm_B.ChangeDutyCycle(0)
sleep(3)


#! Lower the basket

GPIO.output(IN1, GPIO.LOW)  # Set IN1 to HIGH
GPIO.output(IN2, GPIO.HIGH)   # Set IN2 to LOW
GPIO.output(IN3, GPIO.LOW)  # Set IN1 to HIGH
GPIO.output(IN4, GPIO.HIGH)   # Set IN2 to LOW
pwm_A.ChangeDutyCycle(50)    # 50% duty cycle
pwm_B.ChangeDutyCycle(50)

sleep(4.5)

# Stop PWM and clean up
pwm_A.stop()
pwm_B.stop()
GPIO.cleanup()
