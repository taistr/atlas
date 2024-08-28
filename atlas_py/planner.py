from enum import Enum
import time
import numpy as np
import logging
from dataclasses import dataclass

# Dependencies
from atlas_camera import Camera
from serial_comms import SerialComms
from object_detection import ObjectDetection, DetectionResult

# Planner prams
SEARCH_ANGLE = 30
ACCEPTANCE_ANGLE = 5
FIRING_OFFSET = 0.1

# Flags
ATLAS_FINISHED = False

# Configure logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

@dataclass
class Pose():
    x: float
    y: float
    heading: float

@dataclass
class Move():
    angle: float
    distance: float

class State(Enum):
    SEARCHING = 1
    AIMING = 2
    FIRE = 3
    REVERSE = 4
    FINISHED = 5    

class Planner():
    logger: logging.Logger
    state: State
    serial_comms: SerialComms
    object_detector: ObjectDetection
    moves: list[Move]
    last_detection_result = DetectionResult

    def __init__(self):
        self.logger = logging.getLogger(__name__)

        self.state = State.SEARCHING

        self.serial_comms = SerialComms()
        self.object_detector = ObjectDetection()

        self.moves = []
        self.last_detection_result = DetectionResult(False, 0, 0)

        self.logger.info("Planner is f****** initialised!")

    def change_state(self, state: State) -> None:
        """Change the state of the planner"""
        self.logger.info(f"Transitioning to {state} state.")
        self.state = state

    def wait(self, duration: float) -> None:
        """Wait for a set duration"""
        self.logger.info(f"Waiting for {duration} seconds...")
        time.sleep(duration)
        self.logger.info("Wait complete.")

    def run(self) -> None:
        """Main loop for the planner node"""
        while True:
            match self.state:
                case State.SEARCHING:
                    self.last_detection_result = self.object_detector.detect_object()

                    # If an object is detected, transition to the aiming state
                    if self.last_detection_result.detection:
                        self.change_state(State.AIMING)
                    else:
                        self.logger.info("Attempting to turn")
                        move = Move(angle=SEARCH_ANGLE, distance=0)
                        response = self.serial_comms.start_motion( #Get this to block until done
                            heading=move.angle,
                            distance=move.distance
                        )

                        if response:
                            if "MOTION_CONTROLLER" in response and "Turning Complete" in response:
                                pass
                                list.append(self.moves, move)
                        #self.wait(5)
                        
                case State.AIMING:
                    # Try to aim the robot at the detected object
                    move = Move(angle=self.last_detection_result.angle, distance=0)
                    response = self.serial_comms.start_motion(
                        heading=move.angle,
                        distance=move.distance
                    )
                    #self.wait(5)
                    self.last_detection_result = self.object_detector.detect_object()
                    
                    if "MOTION_CONTROLLER" in response and "Straight Complete" in response:
                        if self.last_detection_result.detection and self.last_detection_result.angle < ACCEPTANCE_ANGLE: #! May not be necessary to have it lower
                            # If the object is still detected, transition to the firing state
                            self.change_state(State.FIRE)
                        elif self.last_detection_result.detection and self.last_detection_result.angle > ACCEPTANCE_ANGLE:
                            # If the object is still in view, continue aiming
                            pass
                        else:
                            # If the object is lost, transition back to the searching state
                            self.change_state(State.SEARCHING)
                        
                        self.moves.append(move)

                case State.FIRE:
                    # Drive the robot straight for a set distance
                    move = Move(
                        distance=self.last_detection_result.distance + FIRING_OFFSET,
                        angle=0,
                    )
                    response = self.serial_comms.start_motion(
                        heading=move.angle,
                        distance=move.distance
                    )


                    #self.wait(15) #! This is a guess
                    if "MOTION_CONTROLLER" in response and "Straight Complete" in response:
                        self.change_state(State.REVERSE)
                        self.moves.append(move)

                case State.REVERSE:
                    move = Move(
                        distance=-(self.last_detection_result.distance + FIRING_OFFSET), 
                        angle=0
                    )
                    response = self.serial_comms.start_motion(
                        heading=move.angle,
                        distance=move.distance
                    )


                    #self.wait(15)
                    if "MOTION_CONTROLLER" in response and "Straight Complete" in response:
                        self.change_state(State.FINISHED)
                        self.moves.append(move)
                
                case State.FINISHED:
                    self.logger.info("Atlas has finished")
                    break

def main(args: dict = None):
    planner = Planner()
    planner.run() #TODO: There are some exceptions that are not handled at the moment

if __name__ == "__main__":
    main()
