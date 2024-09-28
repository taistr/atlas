import lgpio
from time import sleep

# Hoisting Control Constants
HOIST_TIME = 4

# LGPIO pin assignments (LGPIO numbering)
IN1 = 17  # Motor A input 1
IN2 = 27  # Motor A input 2
IN3 = 22  # Motor B input 1
IN4 = 23  # Motor B input 2
ENA = 12  # Motor A enable (PWM)
ENB = 13  # Motor B enable (PWM)

# Initialize GPIO
h = lgpio.gpiochip_open(0)  # Open the default GPIO chip

# Set pin modes and initial values for Motor A and Motor B
lgpio.gpio_claim_output(h, IN1, 0)  # Motor A input 1
lgpio.gpio_claim_output(h, IN2, 0)  # Motor A input 2
lgpio.gpio_claim_output(h, IN3, 0)  # Motor B input 1
lgpio.gpio_claim_output(h, IN4, 0)  # Motor B input 2
lgpio.gpio_claim_output(h, ENA, 0)  # Motor A enable (PWM)
lgpio.gpio_claim_output(h, ENB, 0)  # Motor B enable (PWM)

# Set up PWM for Motor A and Motor B
lgpio.tx_pwm(h, ENA, 1000, 0)  # Motor A, 1kHz frequency, 0% duty cycle to start
lgpio.tx_pwm(h, ENB, 1000, 0)  # Motor B, 1kHz frequency, 0% duty cycle to start

# Function to control motor A
def control_motor_a(direction, speed):
    if direction == 'down':
        lgpio.gpio_write(h, IN1, 1)  # Set IN1 HIGH
        lgpio.gpio_write(h, IN2, 0)  # Set IN2 LOW
    elif direction == 'up':
        lgpio.gpio_write(h, IN1, 0)  # Set IN1 LOW
        lgpio.gpio_write(h, IN2, 1)  # Set IN2 HIGH
    else:
        lgpio.gpio_write(h, IN1, 0)  # Stop motor
        lgpio.gpio_write(h, IN2, 0)
    
    lgpio.tx_pwm(h, ENA, 1000, speed)  # Set speed (0 to 100)

# Function to control motor B
def control_motor_b(direction, speed):
    if direction == 'down':
        lgpio.gpio_write(h, IN3, 1)  # Set IN3 HIGH
        lgpio.gpio_write(h, IN4, 0)  # Set IN4 LOW
    elif direction == 'up':
        lgpio.gpio_write(h, IN3, 0)  # Set IN3 LOW
        lgpio.gpio_write(h, IN4, 1)  # Set IN4 HIGH
    else:
        lgpio.gpio_write(h, IN3, 1)  # Stop motor
        lgpio.gpio_write(h, IN4, 1)

    lgpio.tx_pwm(h, ENB, 1000, speed)  # Set speed (0 to 100)

# Example: Move motor A and B forward at half speed
#control_motor_a('up', 50)  # Motor A forward at 50% speed
#control_motor_b('up', 50)  # Motor B forward at 50% speed

sleep(HOIST_TIME)

# Stop both motors
control_motor_a('stop', 0)  # Stop motor A
control_motor_b('stop', 0)  # Stop motor B

sleep(2)

# Example: Move motor A and B forward at half speed
control_motor_a('down', 50)  # Motor A forward at 50% speed
#control_motor_b('down', 50)  # Motor B forward at 50% speed

sleep(HOIST_TIME)

# Stop both motors
control_motor_a('stop', 0)  # Stop motor A
control_motor_b('stop', 0)  # Stop motor B

# Clean up
lgpio.gpiochip_close(h)
