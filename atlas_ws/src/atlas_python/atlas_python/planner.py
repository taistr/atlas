import sys
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType, ParameterDescriptor
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ExternalShutdownException
from atlas_msgs.srv import Detection, MotorCommand
from enum import Enum
import time

class State(Enum):
    SEARCHING = 1
    AIMING = 2
    FIRE = 3
    REVERSE = 4
    FINISHED = 5    


class Planner(Node):
    def __init__(self):
        super().__init__("planner")

        self.state = State.SEARCHING

        self.detection_client = self.create_client(
            Detection,
            "atlas/detect_objects",
        )
        self.motor_client = self.create_client(
            MotorCommand,
            "atlas/motor_command",
        )
        self.last_detection = None
        
        self.get_logger().info("Waiting for services...")
        self.motor_client.wait_for_service()
        self.detection_client.wait_for_service()

        self.get_logger().info("Planner Online!")

    def initialise_parameters(self) -> None:
        """Declare parameters for the planner node"""
        self.declare_parameter(
            "search_angle",
            30,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="Angle to turn when searching for objects (degrees)",
            ),
        )
        self.declare_parameter(
            "acceptance_angle",
            5,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Angle to accept as being aimed at the object (degrees)",
            ),
        )
        self.declare_parameter(
            "firing_offset",
            0.1,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Distance to move forward when firing (m)",
            ),
        )


    def change_state(self, state: State) -> None:
        """Change the state of the planner"""
        self.get_logger().info(f"Transitioning to {state} state.")
        self.state = state

    def wait(self, duration: float) -> None:
        """Wait for a set duration"""
        self.get_logger().info(f"Waiting for {duration} seconds...")
        time.sleep(duration)
        self.get_logger().info("Wait complete.")

    def run_detection(self) -> Detection.Response:
        """Run the object detection service"""
        self.get_logger().info("Running object detection...")
        msg = self.detection_client.call(Detection.Request())
        self.get_logger().info(f"Detection result: {msg.detection}")

    def run(self) -> None:
        """Main loop for the planner node"""

        match self.state:
            case State.SEARCHING:
                self.last_detection = self.run_detection()

                # If an object is detected, transition to the aiming state
                if self.last_detection.detection:
                    self.change_state(State.AIMING)
                else:
                    #Turn the robot 45 degrees
                    self.motor_client.call(
                        MotorCommand.Request(
                            heading=self.get_parameter("search_angle").value,
                            distance=0,
                        )
                    )
                    self.wait(5)
                    
            case State.AIMING:
                # Try to aim the robot at the detected object
                self.motor_client.call(
                    MotorCommand.Request(
                        heading=self.last_detection.angle,
                        distance=0,
                    ) #! Need to consider how controller reacts to micro adjustments
                )
                self.wait(5)
                self.last_detection = self.run_detection()

                acceptance_angle = self.get_parameter("acceptance_angle").value
                if self.last_detection.detection and self.last_detection.angle < acceptance_angle: #! May not be necessary to have it lower
                    # If the object is still detected, transition to the firing state
                    self.change_state(State.FIRE)
                elif self.last_detection.detection and self.last_detection.angle > acceptance_angle:
                    # If the object is still in view, continue aiming
                    pass
                else:
                    # If the object is lost, transition back to the searching state
                    self.change_state(State.SEARCHING)

            case State.FIRE:
                # Drive the robot straight for a set distance
                self.firing_offset = self.get_parameter("firing_offset").value
                self.motor_client.call(
                    MotorCommand.Request(
                        heading=0, 
                        distance=self.last_detection.distance + self.firing_offset,
                    )
                )
                self.wait(15) #! There should be a better way to do this
                self.change_state(State.REVERSE)

            case State.REVERSE:
                self.motor_client.call(
                    MotorCommand.Request(
                        heading=0,
                        distance=-(self.last_detection.distance + self.firing_offset),
                    )
                )
                self.wait(15)
                self.change_state(State.FINISHED)
            
            case State.FINISHED:
                self.get_logger().info("Atlas has finished")


def main(args: dict = None):
    rclpy.init(args=args)
    
    planner = Planner()
    executor = MultiThreadedExecutor()
    try:
        executor.add_node(planner)
        executor.create_task(planner.run)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)

if __name__ == "__main__":
    main()
