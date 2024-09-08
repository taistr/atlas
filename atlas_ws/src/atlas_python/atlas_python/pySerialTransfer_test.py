import serial
from time import sleep

SERIAL_DEVICE = '/dev/ttyACM0'
BAUD_RATE = 115200
SENT_FLAG = False

class motorCmdStruct:
    def __init__(self):
        self.status = 400
        self.cmd = 'b'
        self.arg1 = 0.0
        self.arg2 = 0.0
        self.arg3 = 0.0
        self.arg4 = 0.0

    def to_serial_format(self):
        return f"{self.status}:{self.cmd}:{self.arg1}:{self.arg2}:{self.arg3}:{self.arg4}"

    def from_serial_format(self, serial_data):
        try:
            parts = serial_data.strip().split(':')
            if len(parts) == 6:
                self.status = int(parts[0])
                self.cmd = parts[1]
                self.arg1 = float(parts[2])
                self.arg2 = float(parts[3])
                self.arg3 = float(parts[4])
                self.arg4 = float(parts[5])
        except ValueError:
            print(f"Error parsing serial data: {serial_data}")

if __name__ == '__main__':
    try:
        testStruct = motorCmdStruct()
        ser = serial.Serial(SERIAL_DEVICE, BAUD_RATE, timeout=1)
        
        sleep(2)  # Allow some time for the Arduino to reset

        testStruct.arg1 = 0.0
        testStruct.arg2 = 0.0
        testStruct.status = 100

        while True:
            if not SENT_FLAG:
                print("sending")
                send_str = testStruct.to_serial_format() + "\n"
                ser.write(send_str.encode('utf-8'))
                SENT_FLAG = True
                print(f"Sent: {send_str.strip()}")

            if ser.in_waiting > 0:
                incoming_data = ser.readline().decode('utf-8')
                print(f"Received raw data: {incoming_data.strip()}")
                testStruct.from_serial_format(incoming_data)
                print('RCVD:', testStruct.status, testStruct.cmd, testStruct.arg1, testStruct.arg2, testStruct.arg3, testStruct.arg4)

    except KeyboardInterrupt:
        ser.close()
    except Exception as e:
        print(f"Error: {e}")
        ser.close()
