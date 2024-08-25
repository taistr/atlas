#!/usr/bin/env python3
import serial
import time

BAUD_RATE = 57600

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', BAUD_RATE, timeout=1)
    ser.reset_input_buffer()
    while True:
        ser.write(b"Hello from Raspberry Pi!\n")
        line = ser.readline().decode('utf-8').rstrip()
        print(line)
        time.sleep(1)