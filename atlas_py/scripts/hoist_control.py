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

# Example: Move motor A forward at half speed
GPIO.output(IN1, GPIO.HIGH)  # Set IN1 to HIGH
GPIO.output(IN2, GPIO.LOW)   # Set IN2 to LOW
pwm_A.ChangeDutyCycle(50)    # 50% duty cycle

sleep(2)

# Stop the motor by setting both inputs to HIGH and turning off PWM
GPIO.output(IN1, GPIO.HIGH)
GPIO.output(IN2, GPIO.HIGH)
pwm_A.ChangeDutyCycle(0)  # Set PWM to 0% (stop motor)

# Stop PWM and clean up
pwm_A.stop()
pwm_B.stop()
GPIO.cleanup()
