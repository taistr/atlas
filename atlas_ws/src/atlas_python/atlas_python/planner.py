import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ExternalShutdownException
from atlas_msgs.srv import Detection
from enum import Enum

class State(Enum):
    SEARCHING = 1
    AIMING = 2
    FIRE = 3
    REVERSE = 4


class Planner(Node):
    def __init__(self):
        super().__init__("planner")

        self.state = State.SEARCHING

        self.detection_client = self.create_client(
            Detection,
            "atlas/detect_objects",
        )
        
        self.get_logger().info("Waiting for object detection service...")
        self.detection_client.wait_for_service()

        self.get_logger().info("Planner Online!")

    def change_state(self, state: State) -> None:
        """Change the state of the planner"""
        self.get_logger().info(f"Transitioning to {state} state.")
        self.state = state

    def run(self) -> None:
        """Main loop for the planner node"""

        match self.state:
            case State.SEARCHING:
                detection_response = self.detection_client.call(Detection.Request())

                # If an object is detected, transition to the aiming state
                if detection_response.detection:
                    self.change_state(State.AIMING)
                else:
                    # TODO: Turn 45 degrees to the left
                    pass
            case State.AIMING:
                # TODO: Turn the robot to face the object
                pass
            case State.FIRE:
                # TODO: Drive the robot straight and time the driving
                pass
            case State.REVERSE:
                # TODO: Reverse the robot back for the exact same amount of time
                pass


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
