from hoist import Hoist, HoistMotor, HoistDirection
from time import sleep

# Create an instance of the Hoist class
hoist = Hoist()

try:
    print("Moving motors")

    hoist.deposit()
    
finally:
    # Clean up GPIO and stop PWM
    hoist.cleanup()
    print("Hoist cleanup complete")
