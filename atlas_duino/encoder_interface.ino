#include <Encoder.h>
#include "encoder_interface.h"

// Create Encoder Objects
Encoder leftWheel(LEFT_WHEEL_ENC_A, LEFT_WHEEL_ENC_B);
Encoder rightWheel(RIGHT_WHEEL_ENC_A, RIGHT_WHEEL_ENC_B);

unsigned long previousMicros = 0;
long currentLeftPosition = 0;
long currentRightPosition = 0; 

// Wrapper to read encoder values
long readEncoder(int i){
  unsigned long currentMicros = micros();

    // Get the current encoder positions
  if (i == LEFT){
    currentLeftPosition = leftWheel.read();
    return currentLeftPosition;
  }
  else if (i == RIGHT){
    currentRightPosition = rightWheel.read();
    return currentRightPosition;
  }

  
}

// Reset encoder positions
void resetEncoder(int i) {
  if (i == LEFT){
    currentLeftPosition=0L;
    leftWheel.write(0);
    return;
  } 
  else if (i = RIGHT) { 
    currentRightPosition=0L;
    rightWheel.write(0);
    return;
  }
}


