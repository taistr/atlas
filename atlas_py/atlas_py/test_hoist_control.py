from hoist import Hoist, HoistMotor, HoistDirection
from time import sleep

# Create an instance of the Hoist class
hoist = Hoist()

try:
    print("Moving motors")

    # Raise the hoist up to the ramp
    hoist.move(HoistMotor.MOTOR_FRONT, HoistDirection.UP, 50)
    hoist.move(HoistMotor.MOTOR_REAR, HoistDirection.UP, 50)
    sleep(4.5)

    # Stop the front and rear motors
    hoist.move(HoistMotor.MOTOR_FRONT, HoistDirection.STOP)
    hoist.move(HoistMotor.MOTOR_REAR, HoistDirection.STOP)
    sleep(3)

    # Lower the basket
    hoist.move(HoistMotor.MOTOR_FRONT, HoistDirection.DOWN, 50)
    hoist.move(HoistMotor.MOTOR_REAR, HoistDirection.DOWN, 50)
    sleep(4.5)

finally:
    # Clean up GPIO and stop PWM
    hoist.cleanup()
    print("Hoist cleanup complete")
