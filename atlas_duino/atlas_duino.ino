//SOURCE (for this code and other attached scripts): https://github.com/joshnewans/ros_arduino_bridge/tree/main

#include <Servo.h>

/* Maximum PWM signal */
#define MAX_PWM 255
#define MIN_PWM 50
#define MIN_PWM_ALLOW_ZONE 10

/* Serial */
#define BAUD_RATE 115200

/* Encoder */
#define ENC_TIME_DELTA_THRESHOLD 100000

/* Run the PID loop at 30 times per second */
#define PID_COUNT_MARGIN 50
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
double WHEEL_RADIUS = 56.0/(1000*2); // mm
double WHEEL_BASE = 25.5/100; //cm
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
            resetPID(LEFT);
            resetPID(RIGHT);
            Serial.println("Encoders & PID Reset");
            break;
        case MOTOR_CONTROL:
            // Serial command: m <spdL> <spdR>
            // Sets motors' closed-loop speeds, in m/s
            /* Reset the auto stop timer */
            lastMotorCommand = micros();
            if (arg1d == 0 && arg2d == 0) {
              setMotorSpeeds(0, 0);
              resetPID(LEFT);
              resetPID(RIGHT);
              moving = 0;
              turning = 1;
            }
            else {
              moving = 1;
              turning = 1;
            }

            arg1d = (arg1d/(2*PI*WHEEL_RADIUS)) * 48 * SPINDLE_TO_ENCODER_GEAR_RATIO;  // Distance in m
            //arg2d = (((arg2d/360) * (2*PI*WHEEL_BASE))/(2*PI*WHEEL_RADIUS)) * 48 * SPINDLE_TO_ENCODER_GEAR_RATIO;  // Angle in degrees
            arg2d = (arg2d*(48*SPINDLE_TO_ENCODER_GEAR_RATIO)*(WHEEL_BASE/2))/(360*WHEEL_RADIUS);  // Angle in degrees

            Serial.println(arg2d);
            leftPID.Straight_CountsToTarget = floor(arg1d);
            rightPID.Straight_CountsToTarget = floor(arg1d);
            leftPID.Turning_CountsToTarget = floor(-arg2d);
            rightPID.Turning_CountsToTarget = floor(arg2d);
            break;
        case MOTOR_RAW_PWM:
            /* Reset the auto stop timer */
            lastMotorCommand = micros();
            resetPID(LEFT);
            resetPID(RIGHT);
            moving = 0; // Sneaky way to temporarily disable the PID
            setMotorSpeeds(arg1, arg2);
            Serial.print("OK. Left motor PWM: "); 
            Serial.print(arg1); 
            Serial.print(", Right motor PWM:  ");
            Serial.println(arg2); 
            break; 
        case UPDATE_PID_L_STRAIGHT:
            L_Kp = arg1d;
            L_Ki = arg2d;
            L_Kd = arg3d;
            L_Ko = arg4d;
            Serial.print("OK. LEFT_STRAIGHT PID UPDATED. L_P: ");
            Serial.print(L_Kp);
            Serial.print(", L_I: ");
            Serial.print(L_Ki);
            Serial.print(", L_D: ");
            Serial.print(L_Kd);
            Serial.print(", L_A: ");
            Serial.println(L_Ko);
            break;
        case UPDATE_PID_R_STRAIGHT:
            R_Kp = arg1d;
            R_Ki = arg2d;
            R_Kd = arg3d;
            R_Ko = arg4d;
            Serial.print("OK. RIGHT_STRAIGHT PID UPDATED. R_P: ");
            Serial.print(R_Kp);
            Serial.print(", R_I: ");
            Serial.print(R_Ki);
            Serial.print(", R_D: ");
            Serial.print(R_Kd);
            Serial.print(", R_A: ");
            Serial.println(R_Ko);
            break;
        case UPDATE_PID_L_TURNING:
            L_Kp = arg1d;
            L_Ki = arg2d;
            L_Kd = arg3d;
            L_Ko = arg4d;
            Serial.print("OK. LEFT_TURNING PID UPDATED. L_P: ");
            Serial.print(Lt_Kp);
            Serial.print(", L_I: ");
            Serial.print(Lt_Ki);
            Serial.print(", L_D: ");
            Serial.print(Lt_Kd);
            Serial.print(", L_A: ");
            Serial.println(Lt_Ko);
            break;
        case UPDATE_PID_R_TURNING:
            R_Kp = arg1d;
            R_Ki = arg2d;
            R_Kd = arg3d;
            R_Ko = arg4d;
            Serial.print("OK. RIGHT_TURNING PID UPDATED. R_P: ");
            Serial.print(Rt_Kp);
            Serial.print(", R_I: ");
            Serial.print(Rt_Ki);
            Serial.print(", R_D: ");
            Serial.print(Rt_Kd);
            Serial.print(", R_A: ");
            Serial.println(Rt_Ko);
            break;
          case GET_PID:
            // STRAIGHT
            Serial.print("OK. STRAIGHT. L_P: ");
            Serial.print(L_Kp);
            Serial.print(", L_I: ");
            Serial.print(L_Ki);
            Serial.print(", L_D: ");
            Serial.print(L_Kd);
            Serial.print(", L_A: ");
            Serial.println(L_Ko);
            Serial.print("OK. STRAIGHT. R_P: ");
            Serial.print(R_Kp);
            Serial.print(", R_I: ");
            Serial.print(R_Ki);
            Serial.print(", R_D: ");
            Serial.print(R_Kd);
            Serial.print(", R_A: ");
            Serial.println(R_Ko);
            // TURNING
            Serial.print("OK. TURNING. L_P: ");
            Serial.print(Lt_Kp);
            Serial.print(", L_I: ");
            Serial.print(Lt_Ki);
            Serial.print(", L_D: ");
            Serial.print(Lt_Kd);
            Serial.print(", L_A: ");
            Serial.println(Lt_Ko);
            Serial.print("OK. TURNING. R_P: ");
            Serial.print(Rt_Kp);
            Serial.print(", R_I: ");
            Serial.print(Rt_Ki);
            Serial.print(", R_D: ");
            Serial.print(Rt_Kd);
            Serial.print(", R_A: ");
            Serial.println(Rt_Ko);
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
    resetPID(LEFT);
    resetPID(RIGHT);
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