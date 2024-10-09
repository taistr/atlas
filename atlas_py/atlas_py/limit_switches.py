import RPi.GPIO as GPIO

LEFT_LIMIT_SWITCH_PIN = 26
RIGHT_LIMIT_SWITCH_PIN = 16
LEFTLOWER_LIMIT_SWITCH_PIN = 0
RIGHTLOWER_LIMIT_SWITCH_PIN = 1

class LimitSwitches:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(LEFT_LIMIT_SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(RIGHT_LIMIT_SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(LEFTLOWER_LIMIT_SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(RIGHTLOWER_LIMIT_SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def switches_pressed(self):
        pressed_switches = []

        if not GPIO.input(LEFT_LIMIT_SWITCH_PIN):
            pressed_switches.append(LEFT_LIMIT_SWITCH_PIN)
        if not GPIO.input(RIGHT_LIMIT_SWITCH_PIN):
            pressed_switches.append(RIGHT_LIMIT_SWITCH_PIN)
        if not GPIO.input(LEFTLOWER_LIMIT_SWITCH_PIN):
            pressed_switches.append(LEFTLOWER_LIMIT_SWITCH_PIN)
        if not GPIO.input(RIGHTLOWER_LIMIT_SWITCH_PIN):
            pressed_switches.append(RIGHTLOWER_LIMIT_SWITCH_PIN)

        return pressed_switches  # Return the array directly

    def cleanup(self):
        GPIO.cleanup()