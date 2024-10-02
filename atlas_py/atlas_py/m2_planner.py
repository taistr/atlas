from enum import Enum
import time
import numpy as np
import logging
from dataclasses import dataclass

# Dependencies
from atlas_camera import Camera, FrameGrabber
from serial_comms import SerialComms
from object_detection import ObjectDetection, DetectionResult
from hoist import Hoist

# Planner parameters
SEARCH_ANGLE = 17  # Angle to turn when searching for objects
ACCEPTANCE_ANGLE = 2  # Angle threshold for considering the object aimed at
FIRING_OFFSET = 0.35  # Offset distance when moving towards the object
TENNIS_COURT_CENTRE = 3.42 # metres (distance from corner of a tennis court to centre)
MAX_ONESHOT_DISTANCE = 2 # metres (maximum distance to move in one shot)
BALL_DEPOSIT_THRESHOLD = 3 # balls (number of balls to collect before depositing)
DEFAULT_DELAY_TIME = 2 # seconds (default delay time for waiting)

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
    INITIALISE = 0
    START = 1
    BALL_SEARCH = 2
    BALL_COLLECTION = 3
    BALL_RETURN = 4
    BOX_SEARCH = 5
    DEPOSIT = 6  

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
        self.state = State.START

        self.camera = Camera()
        self.frame_grabber = FrameGrabber(self.camera)
        self.serial_comms = SerialComms()
        self.object_detector = ObjectDetection()
        self.hoist = Hoist()

        self.moves = []
        self.last_detection_result = DetectionResult(False, 0, 0)
        self.skip_aim = False
        self.ball_counter = 0

        self.frame_grabber.start()
        self.logger.info("Planner initialised!")

    def wait(self, duration: float) -> None:
        """
        Pause execution for a specified duration.

        :param duration: Time in seconds to wait.
        """
        self.logger.info(f"Waiting for {duration} seconds...")
        time.sleep(duration)
        self.logger.info("Wait complete.")


    def change_state(self, state: State) -> None:
        """
        Change the current state of the planner.

        :param state: The new state to transition to.
        """
        self.logger.info(f"Transitioning to {state} state.")
        self.state = state

    def run(self) -> None:
        """
        Main loop of the planner, controlling the robot through different states.
        """
        while True:
            match self.state:
                case State.INITIALISE:
                    self.initialise_state()
                case State.START:
                    self.start_state()
                case State.BALL_SEARCH:
                    self.ball_search_state()
                case State.BALL_COLLECTION:
                    self.ball_collection_state()
                case State.BALL_RETURN:
                    self.ball_return_state()
                case State.BOX_SEARCH:
                    self.box_search_state()
                case State.DEPOSIT:
                    self.deposit_state()
                case _:
                    self.logger.error("Invalid state reached. Transitioning to BALL_SEARCH.")
                    self.change_state(State.BALL_SEARCH)



    def initialise_state(self) -> None:
        """
        Atlas waits for a few seconds before starting the state machine.
        """
        WAIT_TIME = 5
        self.logger.info(f"Atlas will be starting in {WAIT_TIME} seconds...")
        time.sleep(WAIT_TIME)
        self.change_state(State.START) # Transition to the next state

    def start_state(self) -> None:
        """
        Atlas drives straight - towards the centre of the arena.
        """
        self.serial_comms.start_motion(
            heading=0,
            distance=TENNIS_COURT_CENTRE
        )
        self.change_state(State.BALL_SEARCH)

    def ball_search_state(self) -> None:
        """
        Atlas searches for a ball.
        """
        self.last_detection_result = self.object_detector.detect_object(
            self.frame_grabber.get_latest_frame()
        )

        # If an object is detected, transition to the aiming state
        if self.last_detection_result.detection:
            self.change_state(State.BALL_COLLECTION)
        else:
            self.logger.info("Attempting to turn")
            self.serial_comms.start_motion(
                heading=SEARCH_ANGLE,
                distance=0
            )
    
    def ball_collection_state(self) -> None:
        """
        Atlas aims at the ball and moves towards it. #TODO: maybe implement as two states
        """
        # aim robot at the nearest ball
        if not self.skip_aim:
            move = Move(angle=self.last_detection_result.angle, distance=0)
            self.serial_comms.start_motion(
                heading=move.angle,
                distance=move.distance
            )
            self.moves.append(move)

            self.skip_aim = True

        self.last_detection_result = self.object_detector.detect_object(
            self.frame_grabber.get_latest_frame()
        )

        if not self.last_detection_result.detection: # TODO: implement a retry mechanism
            self.change_state(State.BALL_SEARCH)
            return
        elif self.last_detection_result and self.last_detection_result.angle < ACCEPTANCE_ANGLE:
            return

        # presumably, we have a ball and it's within the acceptance angle
        if self.last_detection_result.distance <= MAX_ONESHOT_DISTANCE:
            move = Move(
                distance=self.last_detection_result.distance + FIRING_OFFSET,
                angle=0,
            )
            self.serial_comms.start_motion(
                heading=move.angle,
                distance=move.distance
            )
            self.moves.append(move)
            self.change_state(State.BALL_RETURN)
        else:
            move = Move(
                distance=MAX_ONESHOT_DISTANCE,
                angle=0,
            )
            self.serial_comms.start_motion(
                heading=move.angle,
                distance=move.distance
            )
            self.moves.append(move)
            self.skip_aim = True
            return
    
    def ball_return_state(self) -> None:
        """
        Atlas returns the ball to the box.
        """
        # return robot to previous position
        while self.moves:
            move = self.moves.pop()
            self.serial_comms.start_motion(
                heading=-move.angle,
                distance=-move.distance
            )
        
        self.ball_counter += 1
        self.logger.info(f"Ball {self.ball_counter} collected!")

        if self.ball_counter >= BALL_DEPOSIT_THRESHOLD:
            self.change_state(State.BOX_SEARCH)
    
    def box_search_state(self) -> None:
        """
        Atlas searches for the box.
        """
        #TODO: refactor object detector to allow specifying class to detect
        self.last_detection_result = self.object_detector.detect_object(
            self.frame_grabber.get_latest_frame()
        )

        # If an object is detected, transition to the aiming state
        #! Potential to get stuck in this state if box is never detected
        if self.last_detection_result.detection:
            self.change_state(State.DEPOSIT)
        else:
            self.logger.info("Attempting to turn")
            self.serial_comms.start_motion(
                heading=SEARCH_ANGLE,
                distance=0
            )

    def deposit_state(self) -> None:
        """
        Atlas deposits the balls into the box.
        """
        # aim robot at the box #TODO: maybe implement a correction step here like bal_collection_state?
        move = Move(angle=self.last_detection_result.angle, distance=0)
        self.serial_comms.start_motion(
            heading=move.angle,
            distance=move.distance
        )
        self.moves.append(move)

        self.last_detection_result = self.object_detector.detect_object(
            self.frame_grabber.get_latest_frame()
        )

        if not self.last_detection_result.detection: #TODO: implement a retry mechanism
            self.change_state(State.BOX_SEARCH)
            return
        elif self.last_detection_result and self.last_detection_result.angle < ACCEPTANCE_ANGLE:
            self.serial_comms.start_motion(
                heading=0,
                distance=self.last_detection_result.distance + FIRING_OFFSET
            )
            self.hoist.deposit_balls()
            time.sleep(DEFAULT_DELAY_TIME)
            self.serial_comms.start_motion(
                heading=0,
                distance=-self.last_detection_result.distance - FIRING_OFFSET
            )
            self.change_state(State.BALL_SEARCH)
            self.ball_counter = 0
            return 


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
