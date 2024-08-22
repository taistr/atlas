import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import RPi.GPIO as GPIO
except RuntimeError:  # RPi.GPIO throws errors when not on RPi
    from unittest.mock import MagicMock
    GPIO = MagicMock()

IN1 = 9
IN2 = 11
ENA = 13

IN3 = 15
IN4 = 14 # reversed
ENB = 12

class ActuationTest(Node):
    def __init__(self):
        super().__init__("actuation_test")
        self.get_logger().info("Actuation Test Initialising!")

        GPIO.setmode(GPIO.BCM)

        GPIO.setup(IN1, GPIO.OUT)
        GPIO.setup(IN2, GPIO.OUT)
        GPIO.setup(ENA, GPIO.OUT)
        GPIO.setup(IN3, GPIO.OUT)
        GPIO.setup(IN4, GPIO.OUT)
        GPIO.setup(ENB, GPIO.OUT)

        # Set up PWM for motor A
        self.pwm_a = GPIO.PWM(ENA, 100)  # PWM frequency set to 100 Hz
        self.pwm_a.start(0)  # Start PWM with 0% duty cycle

        # Set up PWM for motor B
        self.pwm_b = GPIO.PWM(ENB, 100)  # PWM frequency set to 100 Hz
        self.pwm_b.start(0)  # Start PWM with 0% duty cycle

        self.test_timer = self.create_timer(3, self.test_callback)
        self.flag = False

    def test_callback(self) -> None:
        self.get_logger().info("Test Callback!")
        # if self.flag:
        #     self.set_motor_direction(IN1, IN2, True)
        #     self.set_motor_direction(IN3, IN4, True)
        #     self.set_motor_speed(self.pwm_a, 100)
        #     self.set_motor_speed(self.pwm_b, 100)
        # else:
        #     self.set_motor_direction(IN1, IN2, False)
        #     self.set_motor_direction(IN3, IN4, False)
        #     self.set_motor_speed(self.pwm_a, 75)
        #     self.set_motor_speed(self.pwm_b, 75)

        self.set_motor_direction(IN1, IN2, True)
        self.set_motor_direction(IN3, IN4, True)
        self.set_motor_speed(self.pwm_a, 100)
        self.set_motor_speed(self.pwm_b, 100)

        self.flag = not self.flag

    @staticmethod
    def set_motor_speed(pwm: GPIO.PWM, speed: float):
        pwm.ChangeDutyCycle(speed)

    def set_motor_direction(self, in1: int, in2: int, forward: bool):
        self.get_logger().info("forward" if forward else "backward")
        if forward:
            GPIO.output(in1, GPIO.HIGH)
            GPIO.output(in2, GPIO.LOW)
        else:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.HIGH)

    def cleanup(self):
        self.get_logger().info("Cleaning up GPIO and stopping PWM...")
        self.pwm_a.stop()
        self.pwm_b.stop()
        GPIO.cleanup()

def main(args: dict = None):
    rclpy.init(args=args)
    
    actuation = ActuationTest()
    try:
        actuation.get_logger().info("Starting Actuation Test!")
        rclpy.spin(actuation)
    except KeyboardInterrupt:
        pass
    except rclpy.exceptions.ExternalShutdownException:
        sys.exit(1)
    finally:
        actuation.cleanup()  # Ensure GPIO cleanup happens
        actuation.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
