from serial_comms import SerialComms
import time

def main(args: dict = None):
    serial_comms = SerialComms()
    print("Started")
    response = serial_comms.start_motion(
        heading=0,
        distance=1
    )

    while response:
        print("Finished")
        time.sleep(10)
        serial_comms.cleanup()

main()
