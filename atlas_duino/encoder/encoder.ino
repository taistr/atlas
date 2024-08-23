#include <Encoder.h>

// Pin definitions for the encoders
Encoder leftWheel(2, 4);
Encoder rightWheel(3, 5);

unsigned long previousMicros = 0;

void setup() {
  Serial.begin(115200);  // Set baud rate to match the ROS2 node
}

void loop() {
  unsigned long currentMicros = micros();
  unsigned long timeDelta = currentMicros - previousMicros;

  // Check if the time interval has passed
  if (timeDelta >= 100000) {  // Example threshold: 0.1 seconds
    previousMicros = currentMicros;

    // Get the current encoder positions
    long currentLeftPosition = leftWheel.read();
    long currentRightPosition = rightWheel.read();

    // Send the counts and timeDelta over serial
    Serial.print("L:");
    Serial.print(currentLeftPosition);
    Serial.print(",R:");
    Serial.print(currentRightPosition);
    Serial.print(",DT:");
    Serial.println(timeDelta);  // Send time delta in microseconds
  }
}
