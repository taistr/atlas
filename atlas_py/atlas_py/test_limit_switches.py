import time
import RPi.GPIO as GPIO
from limit_switches import LimitSwitches  

# Create an instance of the LimitSwitches class
limit_switches = LimitSwitches()

try:
    print("Press the limit switches to test...")
    while True:
        # Get the list of pressed switches
        pressed_switches = limit_switches.switches_pressed() 
        
        if pressed_switches:
            print(f"Pressed pins: {', '.join(map(str, pressed_switches))}")
        else:
            print("No limit switches pressed.")

        time.sleep(0.1)  # Small delay to avoid rapid looping

except KeyboardInterrupt:
    print("Test interrupted by the user.")

finally:
    # Clean up GPIO settings before exiting
    limit_switches.cleanup()
    print("GPIO cleanup completed.")