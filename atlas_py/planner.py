import sys
from enum import Enum
import time
import numpy as np

# Dependencies
from motor_driver import motor_command
from object_detection import detect_object

# Planner prams
SEARCH_ANGLE = 30
ACCEPTANCE_ANGLE = 5
FIRING_OFFSET = 0.1

# Flags
ATLAS_FINISHED = False

class State(Enum):
    SEARCHING = 1
    AIMING = 2
    FIRE = 3
    REVERSE = 4
    FINISHED = 5    


class Planner():
    def __init__(self):

        self.state = State.SEARCHING

        self.last_detection = None
        self.moves = np.array([0,0]) 

        print("Planner is fucking Online!")

    def change_state(self, state: State) -> None:
        """Change the state of the planner"""
        print(f"planner.py:Transitioning to {state} state.")
        self.state = state

    def wait(self, duration: float) -> None:
        """Wait for a set duration"""
        print(f"planner.py:Waiting for {duration} seconds...")
        time.sleep(duration)
        print("planner.py:Wait complete.")

    def run_detection(self):
        """Run the object detection service"""
        print("planner.py:Running object detection...")
        response = detect_object

        if response:
            msg = "planner.py:Detection result:Heading_" + str(response["heading"]) + ":Distance_" + str(response["distance"])
            print(msg)
            return response

    def run(self) -> None:
        """Main loop for the planner node"""
    
        match self.state:
            case State.SEARCHING:
                self.last_detection = self.run_detection()

                # If an object is detected, transition to the aiming state
                if self.last_detection["detection"]:
                    self.change_state(State.AIMING)
                else:
                    print("Attempting to turn")
                    move = np.array(SEARCH_ANGLE, 0)
                    motor_command(move[0], move[1])
                    np.vstack((self.moves, move))

                    self.wait(5)
                    pass
                    
            case State.AIMING:
                # Try to aim the robot at the detected object
                move = np.array(self.last_detection["angle"], 0)
                motor_command(move[0], move[1])
                np.vstack((self.moves, move))

                self.wait(5)
                self.last_detection = self.run_detection()

                if self.last_detection["detection"] and self.last_detection["angle"] < ACCEPTANCE_ANGLE: #! May not be necessary to have it lower
                    # If the object is still detected, transition to the firing state
                    self.change_state(State.FIRE)
                elif self.last_detection["detection"] and self.last_detection["angle"] > ACCEPTANCE_ANGLE:
                    # If the object is still in view, continue aiming
                    pass
                else:
                    # If the object is lost, transition back to the searching state
                    self.change_state(State.SEARCHING)

            case State.FIRE:
                # Drive the robot straight for a set distance
                move = np.array(0, self.last_detection["distance"]+FIRING_OFFSET)
                motor_command(move[0], move[1])
                np.vstack((self.moves, move))

                self.wait(15) #! There should be a better way to do this
                self.change_state(State.REVERSE)

            case State.REVERSE:
                move = np.array(0, -(self.last_detection["distance"]+FIRING_OFFSET))
                motor_command(move[0], move[1])
                np.vstack((self.moves, move))

                self.wait(15)
                self.change_state(State.FINISHED)
            
            case State.FINISHED:
                print("planner.py:Atlas has finished")
                ATLAS_FINISHED = True
                return 

def main(args: dict = None):
    planner = Planner()
    try:
        planner.run()
    except:
        print("planner.py:Planner failed")
        sys.exit(1)

if __name__ == "__main__":
    main()
