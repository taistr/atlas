import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from atlas_msgs.srv import Detection, MotionRequest
from enum import Enum

class State(Enum):
    SEARCHING = 1
    AIMING = 2
    FIRE = 3
    REVERSE = 4
    FINISHED = 5   

class Planner(Node):
    def __init__(self):
        super().__init__("planner")
        self.init_params()

        self.state = State.SEARCHING

        self.detection_client = self.create_client(Detection, "atlas/detect_objects")
        self.motion_client = self.create_client(MotionRequest, "atlas/motion_request")   

        self.previous_detection = Detection.Response(
            detection=False,
            angle=0.0,
            distance=0.0,
        )

        self.get_logger().info("Planner f****** online!")

    def change_state(self, state: State) -> None:
        """Change the state of the planner"""
        self.get_logger().info(f"Transitioning to {state.name}")
        self.state = state

    def init_params(self) -> None:
        """Declare parameters for the planner node"""
        self.declare_parameter(
            "search_angle",
            value=15,  # degrees
        )    
        self.declare_parameter(
            "acceptance_angle",
            value=5,  # degrees
        )
        self.declare_parameter(
            "firing_offset",
            value=0.5,  # meters
        )

    def run(self) -> None:
        """Run the planner node"""
        self.get_logger().info("Waiting for object detection service!")
        self.detection_client.wait_for_service()
        self.get_logger().info("Waiting for motion service!")
        self.motion_client.wait_for_service()

        match self.state:
            case State.SEARCHING:
                self.previous_detection = self.detection_client.call(Detection.Request())

                # If an object is detected, transition to aiming
                if self.previous_detection.detection:
                    self.change_state(State.AIMING)
                else:
                    self.get_logger().info("Attempting to turn ...")
                    self.motion_client.call(
                        MotionRequest.Request(
                            angle=self.get_parameter("search_angle").value,
                            distance=0.0,
                        )
                    )
            case State.AIMING:
                # Aim at the detected object
                self.get_logger().info("Aiming at the detected object ...")
                self.motion_client.call(
                    MotionRequest.Request(
                        angle=self.previous_detection.angle,
                        distance=0.0,
                    )
                )

                self.previous_detection = self.detection_client.call(Detection.Request())

                if self.previous_detection.detection and self.previous_detection.angle < self.get_parameter("acceptance_angle").value:
                    # If the object is still detected, transition to firing
                    self.change_state(State.FIRE)
                elif self.previous_detection.detection and self.previous_detection.angle >= self.get_parameter("acceptance_angle").value:
                    # If the object is still detected, keep aiming
                    pass
                else:
                    # If the object is lost, transition to searching
                    self.change_state(State.SEARCHING)
            case State.FIRE:
                self.get_logger().info("Driving at the detected object ...")
                self.motion_client.call(
                    MotionRequest.Request(
                        angle=0.0,
                        distance=self.previous_detection.distance + self.get_parameter("firing_offset").value,
                    )
                )
                self.change_state(State.REVERSE)
            case State.REVERSE:
                self.get_logger().info("Reversing ...")
                self.motion_client.call(
                    MotionRequest.Request(
                        angle=0.0,
                        distance=-(self.previous_detection.distance + self.get_parameter("firing_offset").value),
                    )
                )
                self.change_state(State.FINISHED)
            case State.FINISHED:
                self.get_logger().info("Mission is done!")

        self.executor.create_task(self.run)


def main(args: dict = None):
    rclpy.init(args=args)
    node = Planner()
    executor = MultiThreadedExecutor()
    try:
        executor.add_node(node)
        executor.create_task(node.run)
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    try:
        rclpy.shutdown()
    except rclpy._rclpy_pybind11.RCLError:
        pass

if __name__ == "__main__":
    main()
