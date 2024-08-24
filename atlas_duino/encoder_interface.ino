#include <Encoder.h>
<<<<<<< HEAD
#include "encoder_interface.h"
=======

>>>>>>> 83d976e4ebe29affc36ee7b1983ac1ff0a4bfa73

// // Pin Assignments
// #define LEFT_WHEEL_ENC_A  2
// #define LEFT_WHEEL_ENC_B  4
// #define RIGHT_WHEEL_ENC_A 3
// #define RIGHT_WHEEL_ENC_B 5

// // Serial
// #define BAUD_RATE = 115200

// // Encoder
// #define ENC_TIME_DELTA_THRESHOLD 100000

// Create Encoder Objects
Encoder leftWheel(LEFT_WHEEL_ENC_A, LEFT_WHEEL_ENC_B);
Encoder rightWheel(RIGHT_WHEEL_ENC_A, RIGHT_WHEEL_ENC_B);

unsigned long previousMicros = 0;
long currentLeftPosition = 0;
long currentRightPosition = 0; 

// Wrapper to read encoder values
long readEncoder(int i){
  unsigned long currentMicros = micros();
<<<<<<< HEAD

    // Get the current encoder positions
  if (i == LEFT){
    currentLeftPosition = leftWheel.read();
    return currentLeftPosition;
  }
  else if (i == RIGHT){
    currentRightPosition = rightWheel.read();
    return currentRightPosition;
  }

=======
  unsigned long timeDelta = currentMicros - previousMicros;

  // Check if the time interval has passed
  if (timeDelta >= ENC_TIME_DELTA_THRESHOLD) {  // Example threshold: 0.1 seconds
    previousMicros = currentMicros;
  
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
>>>>>>> 83d976e4ebe29affc36ee7b1983ac1ff0a4bfa73
  
}

// Reset encoder positions
<<<<<<< HEAD
void resetEncoder(int i) {
  if (i == LEFT){
    currentLeftPosition=0L;
    leftWheel.write(0);
    return;
  } 
  else if (i = RIGHT) { 
    currentRightPosition=0L;
    rightWheel.write(0);
=======
void resetEncoders(int i) {
  if (i == LEFT){
    currentLeftPosition=0L;
    return;
  } 
  else if (int i = RIGHT) { 
    currentRightPosition=0L;
>>>>>>> 83d976e4ebe29affc36ee7b1983ac1ff0a4bfa73
    return;
  }
}

// void setup() {
//   Serial.begin(BAUD_RATE);  // Set baud rate to match the ROS2 node
// }


// void loop() {
//   unsigned long currentMicros = micros();
//   unsigned long timeDelta = currentMicros - previousMicros;

//   // Check if the time interval has passed
//   if (timeDelta >= ENC_TIME_DELTA_THRESHOLD) {  // Example threshold: 0.1 seconds
//     previousMicros = currentMicros;

//     // Get the current encoder positions
//     long currentLeftPosition = leftWheel.read();
//     long currentRightPosition = rightWheel.read();

//     // // Send the counts and timeDelta over serial
//     // Serial.print("L:");
//     // Serial.print(-currentLeftPosition);
//     // Serial.print(",R:");
//     // Serial.print(currentRightPosition);
//     // Serial.print(",DT:");
//     // Serial.println(timeDelta);  // Send time delta in microseconds

//     // if (Serial.available()) {
//     //   Serial.read();
//     //   leftWheel.write(0);
//     //   rightWheel.write(0);
//     // }
//   }
// }

