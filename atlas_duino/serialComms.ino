//SOURCE (for this code and other attached scripts): https://github.com/joshnewans/ros_arduino_bridge/tree/main

#include <Servo.h>

/* Maximum PWM signal */
#define MAX_PWM 255

/* Serial */
#define BAUD_RATE 115200

/* Encoder */
#define ENC_TIME_DELTA_THRESHOLD 100000

/* Run the PID loop at 30 times per second */
#define PID_RATE 30     // Hz
/* Convert the rate into an interval */
const long int PID_INTERVAL = 1000L*1000L / PID_RATE;

/* Custom header files */
#include "duino_cmds.h"
#include "encoder_interface.h"
#include "motor_driver.h"
#include "diffbot_motionController.h"

/* Track the next time we make a PID calculation */
unsigned long nextPID = PID_INTERVAL;

/* Stop the robot if it hasn't received a movement command
in this number of microseconds */
//#define AUTO_STOP_INTERVAL 5000000
#define AUTO_STOP_INTERVAL 10000000
long lastMotorCommand = AUTO_STOP_INTERVAL;

//    Initialising variables
// Serial ---------------------------------------------------
//Variables to help parse serial commands
int arg = 0;
int idx = 0;

//Variable to hold serial command character
char cmd;

//Variable to hold serial data
char chr;

//Variables to hold serial command arguments
char argv1[16];
char argv2[16];
char argv3[16];
char argv4[16];

//Variables to hold serial command arguments integer codes
long arg1;
long arg2;
long arg3;
long arg4;

//Variables to hold serial command arguments' doubles
double arg1d;
double arg2d;
double arg3d;
double arg4d;
//---------------------------------------------------------
// Differential Drive--------------------------------------
double SPINDLE_TO_ENCODER_GEAR_RATIO = 74.8317;
double WHEEL_RADIUS = 56.0/(1000*2);
double WHEEL_BASE = 25.5/1000;
//---------------------------------------------------------


void resetCmd(){
    cmd = NULL;
    memset(argv1, 0, sizeof(argv1));
    memset(argv2, 0, sizeof(argv2));
    memset(argv3, 0, sizeof(argv3));
    memset(argv4, 0, sizeof(argv4));
    arg1 = 0;
    arg2 = 0;
    arg3 = 0;
    arg4 = 0;
    arg = 0;
    idx = 0;
}

int executeSerialCmd(){
    int i = 0;
    char *p = argv1;
    char *str;
    int pid_args[4];

    arg1d = atof(argv1);
    arg2d = atof(argv2);
    arg3d = atof(argv3);
    arg4d = atof(argv4);
    

    if ((arg1d - floor(arg1d))==0){
      arg1 = atoi(argv1);
    } 

    if ((arg2d - floor(arg2d))==0){
      arg2 = atoi(argv2);
    } 

    if ((arg3d - floor(arg3d))==0){
      arg3 = atoi(argv3);
    } 

    if ((arg4d - floor(arg4d))==0){
      arg4 = atoi(argv4);
    } 

    
    switch(cmd){
        case SET_WHEEL_RADIUS:
            WHEEL_RADIUS = arg1d;
            Serial.print("OK. WHEEL_RADIUS set to: ");
            Serial.println(WHEEL_RADIUS);
            break;
        case SET_MOTOR_RATIO:
            SPINDLE_TO_ENCODER_GEAR_RATIO = arg1d;
            Serial.print("OK. SPINDLE_TO_ENCODER_GEAR_RATIO set to: ");
            Serial.println(SPINDLE_TO_ENCODER_GEAR_RATIO);
            break;
        case GET_BAUD_RATE:
            Serial.println(BAUD_RATE);
            break;
        case GET_TIMESTAMP:
            Serial.println(micros());
            break;
        case ANALOG_READ:
            Serial.println(analogRead(arg1));
            break;
        case DIGITAL_READ:
            Serial.println(digitalRead(arg1));
            break;
        case ANALOG_WRITE:
            char data [16];
            analogWrite(arg1, arg2);
            sprintf(data, "OK. Analog write to pin %3d with value %3d ",arg1, arg2);
            Serial.println(data);
            break;
        case PIN_MODE:
            if (arg2 == 0) pinMode(arg1, INPUT);
            else if (arg2 == 1) pinMode(arg1, OUTPUT);
            Serial.println("OK");
            break;
        case PING:
            //Serial.println(Ping(arg1));
            break;
        case SERVO_WRITE:
            // servos[arg1].setTargetPosition(arg2);
            // Serial.println("OK");
            break;
        case SERVO_READ:
            // Serial.println(servos[arg1].getServo().read());
            break;
        case READ_ENCODERS:
            // Serial command: e
            // Send the counts and timestamp (microseconds) over serial
            Serial.print("L:");
            Serial.print(-readEncoder(LEFT));
            Serial.print(",R:");
            Serial.print(readEncoder(RIGHT));
            Serial.print(",Timestamp:");
            Serial.println(micros());  // Send time delta in microseconds
            resetEncoder(LEFT);
            resetEncoder(RIGHT);
            break;
        case RESET_ENCODERS:
            resetEncoder(LEFT);
            resetEncoder(RIGHT);
            resetPID();
            Serial.println("Encoders & PID Reset");
            break;
        case MOTOR_CONTROL:
            // Serial command: m <spdL> <spdR>
            // Sets motors' closed-loop speeds, in m/s
            /* Reset the auto stop timer */
            lastMotorCommand = micros();
            if (arg1d == 0 && arg2d == 0) {
              setMotorSpeeds(0, 0);
              resetPID();
              moving = 0;
              turning = 1;
            }
            else {
              moving = 1;
              turning = 1;
            }


            //arg1d = (arg1d/(2*PI*WHEEL_RADIUS)) * 48 * SPINDLE_TO_ENCODER_GEAR_RATIO;
            //arg2d = (arg2d/(2*PI*WHEEL_RADIUS)) * 48 * SPINDLE_TO_ENCODER_GEAR_RATIO;

            arg1d = (arg1d/(2*PI*WHEEL_RADIUS)) * 48 * SPINDLE_TO_ENCODER_GEAR_RATIO;  // Distance in m
            arg2d = (((arg2d/360) * (2*PI*WHEEL_BASE))/(2*PI*WHEEL_RADIUS)) * 48 * SPINDLE_TO_ENCODER_GEAR_RATIO;  // Angle in degrees

            //leftPID.TargetTicksPerFrame = arg1d;
            //rightPID.TargetTicksPerFrame = arg2d;
            leftPID.Straight_CountsToTarget = floor(arg1d);
            rightPID.Straight_CountsToTarget = floor(arg1d);
            leftPID.Turning_CountsToTarget = floor(arg2d);
            rightPID.Turning_CountsToTarget = floor(-arg2d);

            //Serial.print("OK. Turning mode. Left motor: "); 
            //Serial.print(leftPID.Turning_CountsToTarget);
            //Serial.print(" Counts, Right motor: ");
            //Serial.print(rightPID.Turning_CountsToTarget);
            //Serial.println(" Counts");
            
            //Serial.print("OK. Straight line mode. Left motor: "); 
            //Serial.print(leftPID.Straight_CountsToTarget);
            //Serial.print(" Counts, Right motor: ");
            //Serial.print(rightPID.Straight_CountsToTarget);
            //Serial.println(" Counts");
            break;
        case MOTOR_RAW_PWM:
            /* Reset the auto stop timer */
            lastMotorCommand = micros();
            resetPID();
            moving = 0; // Sneaky way to temporarily disable the PID
            setMotorSpeeds(arg1, arg2);
            Serial.print("OK. Left motor PWM: "); 
            Serial.print(arg1); 
            Serial.print(", Right motor PWM:  ");
            Serial.println(arg2); 
            break; 
        case UPDATE_PID:
            Kp = arg1d;
            Ki = arg2d;
            Kd = arg3d;
            Ko = arg4d;
            Serial.print("OK. PID UPDATED. P: ");
            Serial.print(Kp);
            Serial.print(", I: ");
            Serial.print(Ki);
            Serial.print(", D: ");
            Serial.print(Kd);
            Serial.print(", A: ");
            Serial.println(Ko);
            break;
          case GET_PID:
            Serial.print("OK. P: ");
            Serial.print(Kp);
            Serial.print(", I: ");
            Serial.print(Ki);
            Serial.print(", D: ");
            Serial.print(Kd);
            Serial.print(", A: ");
            Serial.println(Ko);
            break;
        default:
            Serial.println("Invalid Command");
            break;
        
    }
}


/* Setup function--runs once at startup. */
void setup() {
    Serial.begin(BAUD_RATE);
    initMotorController();
    resetPID();
}

void loop(){
    while (Serial.available() > 0) {
    
        // Read the next character
        chr = Serial.read();

        // Terminate a command with a CR
        if (chr == 13) {
            if (arg == 1) {
                argv1[idx] = NULL;
            
            }
            else if (arg == 2){
                 argv2[idx] = NULL;
            }
            else if (arg == 3){
                 argv3[idx] = NULL;
            }
            else if (arg == 4){
                 argv4[idx] = NULL;
            }
            executeSerialCmd();
            resetCmd();
        }
        // Use spaces to delimit parts of the command
        else if (chr == ' ') {
            // Step through the arguments
            if (arg == 0) arg = 1;
            else if (arg == 1)  {
                argv1[idx] = NULL;
                arg = 2;
                idx = 0;
            }
            else if (arg == 2)  {
                argv2[idx] = NULL;
                arg = 3;
                idx = 0;
            }
            else if (arg == 3)  {
                argv3[idx] = NULL;
                arg = 4;
                idx = 0;
            }
            continue;
        }
        else {
            if (arg == 0) {
                // The first arg is the single-letter command
                cmd = chr;
            }
            else if (arg == 1) {
                // Subsequent arguments can be more than one character
                argv1[idx] = chr;
                idx++;
            }
            else if (arg == 2) {
                argv2[idx] = chr;
                idx++;
            }
            else if (arg == 3) {
                argv3[idx] = chr;
                idx++;
            }
            else if (arg == 4) {
                argv4[idx] = chr;
                idx++;
            }
        }
    }
  
    // run a PID calculation at the appropriate intervals
    if (micros() > nextPID) {
        updatePID();
        nextPID += PID_INTERVAL;
    }
    
    // Check to see if we have exceeded the auto-stop interval
    //if (!moving && ((micros() - lastMotorCommand) > AUTO_STOP_INTERVAL)) {
        //Serial.println("AUTOSTOPPPED");
    //    setMotorSpeeds(0, 0);
    //    moving = 0;
    //}


    //int i;
    //for (i = 0; i < N_SERVOS; i++) {
    //    servos[i].doSweep();
    //}
}