#include "motor_driver.h"

void initMotorController() {
  digitalWrite(LEFT_MOTOR_EN, LOW);
  digitalWrite(LEFT_MOTOR_IN1, LOW); 
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_EN, LOW);
  digitalWrite(RIGHT_MOTOR_IN3, LOW); 
  digitalWrite(RIGHT_MOTOR_IN4, LOW);
}

void setMotorSpeed(int i, int spd) {
  unsigned char reverse = 0;

  if (spd < 0)
  {
    spd = -spd;
    reverse = 1;
  }
  if (spd > 255)
    spd = 255;
  
  if (i == LEFT) { 
    if (reverse == 0 && spd != 0) { 
      analogWrite(LEFT_MOTOR_EN, spd);
      digitalWrite(LEFT_MOTOR_IN1, HIGH); 
      digitalWrite(LEFT_MOTOR_IN2, LOW); 
      }
    else if (reverse == 1 && spd != 0) {
      analogWrite(LEFT_MOTOR_EN, spd);
      digitalWrite(LEFT_MOTOR_IN1, LOW); 
      digitalWrite(LEFT_MOTOR_IN2, HIGH);
      }
    else if (spd == 0){
      digitalWrite(LEFT_MOTOR_EN, HIGH);
      digitalWrite(LEFT_MOTOR_IN1, LOW); 
      digitalWrite(LEFT_MOTOR_IN2, LOW);
    }
  }
  else if (i == RIGHT) {
    if (reverse == 0 && spd != 0) {
      analogWrite(RIGHT_MOTOR_EN, spd);
      digitalWrite(RIGHT_MOTOR_IN3, LOW); 
      digitalWrite(RIGHT_MOTOR_IN4, HIGH); 
      }
    else if (reverse == 1 && spd != 0) { 
      analogWrite(RIGHT_MOTOR_EN, spd);
      digitalWrite(RIGHT_MOTOR_IN3, HIGH); 
      digitalWrite(RIGHT_MOTOR_IN4, LOW);  
      }
    else if (spd == 0){
      digitalWrite(RIGHT_MOTOR_EN, HIGH);
      digitalWrite(RIGHT_MOTOR_IN3, LOW); 
      digitalWrite(RIGHT_MOTOR_IN4, LOW);
    }
  }
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}