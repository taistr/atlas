# Generic Imports
import time
from threading import Lock, Thread, Event
import sys
from dataclasses import dataclass
import asyncio
import signal


# Dependencies
#from serialComms import serial_send

# Robot Params
WHEEL_RADIUS = (56/1000)/2 #metres
WHEEL_BASE = 25.5/100 #metres

# PID Params
#Straight
L_KP = 0.1
L_KI = 0.05
L_KD = 5
L_KO = 10
R_KP = 0.1
R_KI = 0.05
R_KD = 5
R_KO = 10
#Turning
LT_KP = 0.1
LT_KI = 0
LT_KD = 5
LT_KO = 10
RT_KP = 0.1
RT_KI = 0
RT_KD = 5
RT_KO = 10


# Flags
CLEANUP = False


class MotorDriver():
    def __init__(self):
        # Async Set up
        self.future = asyncio.Future()

        # Motor Command blocker flag
        self.motor_command_complete = False

        print("motor_driver.py:Motor drivers ON zoomzoom")
        

    def start_motion(self, heading, distance):
        cmd_string = "m " + str(distance) + " " + str(heading)
        serial_send("motor_driver", cmd_string)


    def cleanup(self):
        print("motor_driver:motor_driver peaceing out")
        serial_send("motor_driver", "o 0 0")


#def motorComplete_callback(response):
#   print("motor_driver.py:Motor Serial Response:" + str(response))


def signal_handler(sig, frame):
    CLEANUP = True
    sys.exit(0)


def motor_command(heading, distance):    
    motor_driver = MotorDriver()

    motor_driver.start_motion(heading, distance)

    signal.signal(signal.SIGINT, signal_handler)
    #signal.pause()

    if CLEANUP:
        motor_driver.cleanup()

    



