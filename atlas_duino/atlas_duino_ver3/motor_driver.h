/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#define RIGHT_MOTOR_IN3 12
#define RIGHT_MOTOR_IN4  13
#define LEFT_MOTOR_IN1  9
#define LEFT_MOTOR_IN2  8
#define RIGHT_MOTOR_EN 11
#define LEFT_MOTOR_EN 10

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);