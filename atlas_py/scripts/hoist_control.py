import RPi.LGPIO as LGPIO
from time import sleep

# LGPIO pin assignments (LGPIO numbering)
IN1 = 17
IN2 = 27
IN3 = 22
IN4 = 23
ENA = 12
ENB = 13

# Initialize LGPIO
h = LGPIO.gpiochip_open(0)  # Open the default GPIO chip

# Set pin modes
LGPIO.gpio_claim_output(h, IN1)
LGPIO.gpio_claim_output(h, IN2)
LGPIO.gpio_claim_output(h, IN3)
LGPIO.gpio_claim_output(h, IN4)
LGPIO.gpio_claim_output(h, ENA)
LGPIO.gpio_claim_output(h, ENB)

# Set up PWM
LGPIO.gpio_pwm(h, ENA, 1000, 0)  # 1kHz PWM frequency, 0% duty cycle
LGPIO.gpio_pwm(h, ENB, 1000, 0)

# Example: Move motor A forward at half speed
LGPIO.gpio_write(h, IN1, 1)  # HIGH in RPi.GPIO is 1 in RPi.LGPIO
LGPIO.gpio_write(h, IN2, 0)  # LOW in RPi.GPIO is 0 in RPi.LGPIO
LGPIO.gpio_pwm(h, ENA, 500)  # 50% duty cycle (range 0-1000)

sleep(2)

LGPIO.gpio_write(h, IN1, 1)  # HIGH in RPi.GPIO is 1 in RPi.LGPIO
LGPIO.gpio_write(h, IN2, 1)  # LOW in RPi.GPIO is 0 in RPi.LGPIO
LGPIO.gpio_pwm(h, ENA, 0)  # 50% duty cycle (range 0-1000)

# ... your motor control logic here ...

# Clean up
LGPIO.gpiochip_close(h)