import RPi.GPIO as GPIO

LEFT_LIMIT_SWITCH_PIN = 26
RIGHT_LIMIT_SWITCH_PIN = 16

class LimitSwitches:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(LEFT_LIMIT_SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(RIGHT_LIMIT_SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def switches_pressed(self):
        return not GPIO.input(LEFT_LIMIT_SWITCH_PIN) or not GPIO.input(RIGHT_LIMIT_SWITCH_PIN)

    def cleanup(self):
        GPIO.cleanup()
