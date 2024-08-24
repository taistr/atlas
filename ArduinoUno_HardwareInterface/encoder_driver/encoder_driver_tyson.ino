#include <Encoder.h>
#include "encoder_driver.h"

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder leftWheel(LEFT_ENC_PIN_A, LEFT_ENC_PIN_B);
Encoder rightWheel(RIGHT_ENC_PIN_A, RIGHT_ENC_PIN_B);

void setup() {
  Serial.begin(9600);
  Serial.println("Two Wheels Encoder Test:");
}

long positionLeft  = -999;
long positionRight = -999;

void loop() {
  long newLeft, newRight;
  newLeft = leftWheel.read();
  newRight = rightWheel.read();

  if (newLeft != positionLeft || newRight != positionRight) {
    Serial.print("Left = ");
    Serial.print(newLeft);
    Serial.print(", Right = ");
    Serial.print(newRight);
    Serial.println();
    positionLeft = newLeft;
    positionRight = newRight;
  }

  // if a character is sent from the serial monitor,
  // reset both back to zero.
  if (Serial.available()) {
    Serial.read();
    Serial.println("Reset both knobs to zero");
    leftWheel.write(0);
    rightWheel.write(0);
  }
}