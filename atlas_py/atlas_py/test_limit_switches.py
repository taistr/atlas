import time
import RPi.GPIO as GPIO
from limit_switches import LimitSwitches  # Replace 'your_module' with the actual module name

# Create an instance of the LimitSwitches class
limit_switches = LimitSwitches()

try:
    print("Press the limit switches to test...")
    while True:
        # Check if any limit switch is pressed
        if limit_switches.switches_pressed():
            print("switch activated")
        else:
            print("No limit switches pressed.")

        time.sleep(0.1)  # Small delay to avoid rapid looping

except KeyboardInterrupt:
    print("Test interrupted by the user.")

finally:
    # Clean up GPIO settings before exiting
    limit_switches.cleanup()
    print("GPIO cleanup completed.")
