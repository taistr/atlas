from enum import Enum
import time
import numpy as np
import logging
from dataclasses import dataclass

# Dependencies
from atlas_camera import Camera, FrameGrabber
from serial_comms import SerialComms
from object_detection import ObjectDetection, DetectionResult

# Planner parameters
SEARCH_ANGLE = 17  # Angle to turn when searching for objects
ACCEPTANCE_ANGLE = 2  # Angle threshold for considering the object aimed at
FIRING_OFFSET = 0.35  # Offset distance when moving towards the object

# Configure logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

@dataclass
class Pose:
    """
    Represents the position and heading of the robot.

    Attributes:
        x (float): X-coordinate of the robot's position.
        y (float): Y-coordinate of the robot's position.
        heading (float): The heading angle of the robot in degrees.
    """
    x: float
    y: float
    heading: float

@dataclass
class Move:
    """
    Represents a movement command for the robot.

    Attributes:
        angle (float): The angle to turn before moving.
        distance (float): The distance to move in meters.
    """
    angle: float
    distance: float

class State(Enum):
    """
    Enumeration of states for the planner.

    Attributes:
        SEARCHING (int): State where the robot is searching for an object.
        AIMING (int): State where the robot is aiming at the detected object.
        FIRE (int): State where the robot moves towards the object.
        REVERSE (int): State where the robot reverses after firing.
        FINISHED (int): State indicating the robot has completed its task.
    """
    SEARCHING = 1
    AIMING = 2
    FIRE = 3
    REVERSE = 4
    FINISHED = 5    

class Planner:
    """
    Main planning class for controlling the robot's behavior.

    Attributes:
        logger (logging.Logger): Logger for the planner.
        state (State): Current state of the planner.
        serial_comms (SerialComms): Serial communication handler for sending commands to the robot.
        object_detector (ObjectDetection): Object detection module for identifying targets.
        moves (list[Move]): List of movements executed by the planner.
        last_detection_result (DetectionResult): Last detection result from the object detector.
    """
    logger: logging.Logger
    state: State
    serial_comms: SerialComms
    object_detector: ObjectDetection
    moves: list[Move]
    last_detection_result = DetectionResult

    def __init__(self):
        """
        Initializes the Planner, setting up camera, serial communication, and object detection.
        """
        self.logger = logging.getLogger(__name__)
        self.state = State.SEARCHING

        self.camera = Camera()
        self.frame_grabber = FrameGrabber(self.camera)
        self.serial_comms = SerialComms()
        self.object_detector = ObjectDetection()

        self.moves = []
        self.last_detection_result = DetectionResult(False, 0, 0)

        self.frame_grabber.start()
        self.logger.info("Planner initialised!")

    def change_state(self, state: State) -> None:
        """
        Change the current state of the planner.

        :param state: The new state to transition to.
        """
        self.logger.info(f"Transitioning to {state} state.")
        self.state = state

    def wait(self, duration: float) -> None:
        """
        Pause execution for a specified duration.

        :param duration: Time in seconds to wait.
        """
        self.logger.info(f"Waiting for {duration} seconds...")
        time.sleep(duration)
        self.logger.info("Wait complete.")

    def run(self) -> None:
        """
        Main loop of the planner, controlling the robot through different states.
        """
        while True:
            match self.state:
                case State.SEARCHING:
                    self.last_detection_result = self.object_detector.detect_object(
                        self.frame_grabber.get_latest_frame()
                    )

                    # If an object is detected, transition to the aiming state
                    if self.last_detection_result.detection:
                        self.change_state(State.AIMING)
                    else:
                        self.logger.info("Attempting to turn")
                        move = Move(angle=SEARCH_ANGLE, distance=0)
                        self.serial_comms.start_motion(
                            heading=move.angle,
                            distance=move.distance
                        )
                        self.moves.append(move)
                        
                case State.AIMING:
                    # Try to aim the robot at the detected object
                    move = Move(angle=self.last_detection_result.angle, distance=0)
                    self.serial_comms.start_motion(
                        heading=-move.angle,
                        distance=move.distance
                    )
                    self.moves.append(move)
                    self.last_detection_result = self.object_detector.detect_object(
                        self.frame_grabber.get_latest_frame()
                    )

                    if self.last_detection_result.detection and self.last_detection_result.angle < ACCEPTANCE_ANGLE:
                        # If the object is still detected and within an acceptable angle, transition to firing
                        self.change_state(State.FIRE)
                    elif self.last_detection_result.detection and self.last_detection_result.angle > ACCEPTANCE_ANGLE:
                        # If the object is detected but not aligned, continue aiming
                        pass
                    else:
                        # If the object is lost, transition back to searching
                        self.change_state(State.SEARCHING)

                case State.FIRE:
                    # Drive the robot straight towards the detected object
                    move = Move(
                        distance=self.last_detection_result.distance + FIRING_OFFSET,
                        angle=0,
                    )
                    self.serial_comms.start_motion(
                        heading=move.angle,
                        distance=move.distance
                    )
                    self.moves.append(move)
                    self.change_state(State.REVERSE)

                case State.REVERSE:
                    # Reverse the robot back to its starting position
                    move = Move(
                        distance=-(self.last_detection_result.distance + FIRING_OFFSET), 
                        angle=0
                    )
                    self.serial_comms.start_motion(
                        heading=move.angle,
                        distance=move.distance
                    )
                    self.moves.append(move)
                    self.change_state(State.FINISHED)
                
                case State.FINISHED:
                    self.logger.info("Atlas has finished")
                    break

def main(args: dict = None):
    try:
        planner = Planner()
        planner.run()  # TODO: There are some exceptions that are not handled at the moment
    except KeyboardInterrupt:
        logging.info("Keyboard interrupt received. Stopping gracefully...")
    except Exception as e:
        logging.error(f"An error occurred: {e}")
    finally:
        planner.frame_grabber.stop()  # Ensure cleanup on exit

if __name__ == "__main__":
    main()
