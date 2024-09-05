//SOURCE (for this code and other attached scripts): https://github.com/joshnewans/ros_arduino_bridge/tree/main

#include <Servo.h>
#include "SerialTransfer.h"

/* Serial params */
#define BAUD_RATE 115200


/* PWM params */
/* Maximum PWM signal */
#define MAX_PWM 255
/* Minimum PWM signal */
#define MIN_PWM 50    //Equivalent to 19.6% duty cycle
/* Range below MIN_PWM at which PWM signal is locked to MIN_PWM - To allow for motor micro-adjustments*/
#define MIN_PWM_ALLOW_ZONE  20


/* PID params */
/* Run the PID loop at 30 times per second */
#define PID_RATE 30     // Hz
/* Convert the rate into an interval */
const long int PID_INTERVAL = 1000L*1000L / PID_RATE;
/* PID error margin - anything under margin is considered perfect */
#define PID_ERROR_MARGIN 50
/* Track the next time we make a PID calculation */
unsigned long nextPID = PID_INTERVAL;


/* Stop the robot if it hasn't received a movement command
in this number of microseconds */
#define AUTO_STOP_INTERVAL 30000000 //Equivalent to 30 seconds   
long lastMotorCommand = AUTO_STOP_INTERVAL;


//    Initialising variables
// Serial ---------------------------------------------------
//Variables to help parse serial commands
int arg = 0;
int idx = 0;

//Variable to hold serial command character
//char cmd;

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

// Serial control flags
bool RX_ALLOWED = true;
bool TX_ALLOWED = true;
bool TX_REQUESTED = false;

// Initialising pySerialTransfer class
SerialTransfer serialInterface;

// Serial Command Structs
struct __attribute__((packed)) STRUCT_RX {
  uint16_t status;
  char cmd;
  float arg1;
  float arg2;
  float arg3;
  float arg4;
} serialRxStruct;

struct __attribute__((packed)) STRUCT_TX {
  uint16_t status;
  char cmd;
  float arg1;
  float arg2;
  float arg3;
  float arg4;
} serialTxStruct;
//---------------------------------------------------------
// Differential Drive--------------------------------------
double SPINDLE_TO_ENCODER_GEAR_RATIO = 74.8317;
double WHEEL_RADIUS = 56.0/(1000*2); // mm
double WHEEL_BASE = 25.5/100; //cm
//---------------------------------------------------------

/* Custom header files */
#include "duino_cmds.h"
#include "encoder_interface.h"
#include "motor_driver.h"
#include "diffbot_motionController.h"


/* 
  Utility functions to check data types.

  A bit of a hacky solution, using function overloading in C++
*/
String types(String a) { return("String"); }
String types(int a) { return("int"); }
String types(char *a) { return("char"); }
String types(float a) { return("float"); }
String types(bool a) { return("bool"); }

/* 
  Resets all serial command variables 
*/
void resetCmd(){
    serialRxStruct.cmd = NULL;
    serialRxStruct.arg1 = 0;
    serialRxStruct.arg2 = 0;
    serialRxStruct.arg3 = 0;
    serialRxStruct.arg4= 0;
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


/* 
  Executes a selected command, using provided arguments 
      Command format: <command letter> <arg1> <arg2> <arg3> <arg4>

      Command letters:
      ANALOG_READ               'a'
      GET_BAUD_RATE             'b'
      PIN_MODE                  'c'
      DIGITAL_READ              'd'
      READ_ENCODERS             'e'
      GET_TIMESTAMP             'f'
      SET_WHEEL_RADIUS          'g'
      SET_MOTOR_RATIO           'h'
      GET_PID                   'i'
      MOTOR_CONTROL             'm'
      MOTOR_RAW_PWM             'o'
      PING                      'p'
      RESET_ENCODERS            'r'
      SERVO_WRITE               's'
      SERVO_READ                't'
      UPDATE_PID_L_STRAIGHT     'y'
      UPDATE_PID_R_STRAIGHT     'u'
      UPDATE_PID_L_TURNING      '6'
      UPDATE_PID_R_TURNING      '7'
      DIGITAL_WRITE             'w'
      ANALOG_WRITE              'x'
*/
int executeSerialCmd(){

    /* Internal serial command parsing variables */
    int i = 0;
    char *p = argv1;
    char *str;
    int pid_args[4];

    /* Set floating point arguments first */
    arg1d = atof(argv1);
    arg2d = atof(argv2);
    arg3d = atof(argv3);
    arg4d = atof(argv4);
    
    serialTxStruct.status = 400;  //Failed

    
    switch(serialRxStruct.cmd){
        case SET_WHEEL_RADIUS:
        /* g <WHEEL_RADIUS> */
            if (types(serialRxStruct.arg1) == "float"){
              WHEEL_RADIUS = serialRxStruct.arg1;

              serialTxStruct.cmd = 'g';
              serialTxStruct.arg1 = WHEEL_RADIUS;
              serialTxStruct.status = 200;   //Successful
              break;
            }
            else{
              break;    //Failed
            }
            
        case SET_MOTOR_RATIO:
        /* h <SPINDLE_TO_ENCODER_GEAR_RATIO> */
            if (types(serialRxStruct.arg1) == "float"){
              SPINDLE_TO_ENCODER_GEAR_RATIO = serialRxStruct.arg1;

              serialTxStruct.cmd = 'h';
              serialTxStruct.arg1 = SPINDLE_TO_ENCODER_GEAR_RATIO;
              serialTxStruct.status = 200;   //Successful
              break;
            }
            else{
              break;   //Failed
            }
        case GET_BAUD_RATE:
        /* b */
            digitalWrite(LED_BUILTIN, HIGH);
            serialTxStruct.cmd = 'a'; 
            serialTxStruct.arg1 = BAUD_RATE;
            serialTxStruct.arg2 = 0.0;
            serialTxStruct.arg3 = 0.0;
            serialTxStruct.arg4 = 0.0;
            serialTxStruct.status = 200;   //Successful
            break;
        case GET_TIMESTAMP:
        /* f */
            serialTxStruct.cmd = 'f';   
            serialTxStruct.arg1 = micros();
            serialTxStruct.status = 200;   //Successful
            break;
        case ANALOG_READ:
        /* a <IDE_PIN_NUM> */
            if (types(serialRxStruct.arg1) == "float" && (serialRxStruct.arg1 - floor(serialRxStruct.arg1)) == 0){
              int pin = int(serialRxStruct.arg1);

              serialTxStruct.cmd = 'a';
              serialTxStruct.arg1 = float(analogRead(pin));
              serialTxStruct.status = 200;   //Successful
              break;
            }
            else{
              break;   //Failed
            }
        case DIGITAL_READ:
        /* d <IDE_PIN_NUM> */
            if (types(serialRxStruct.arg1) == "float" && (serialRxStruct.arg1 - floor(serialRxStruct.arg1)) == 0){
              int pin = int(serialRxStruct.arg1);

              serialTxStruct.cmd = 'd';
              serialTxStruct.arg1 = float(digitalRead(pin));
              serialTxStruct.status = 200;   //Successful
              break;
            }
            else{
              break;    //Failed
            }
        case ANALOG_WRITE:
        /* x <IDE_PIN_NUM> */
            if (types(serialRxStruct.arg1) == "float" && types(serialRxStruct.arg2) == "float" 
            && (serialRxStruct.arg1 - floor(serialRxStruct.arg1)) == 0 && (serialRxStruct.arg2 - floor(serialRxStruct.arg2)) == 0){
              int pin = int(serialRxStruct.arg1);
              int val = int(serialRxStruct.arg2);
              analogWrite(pin, val);

              serialTxStruct.cmd = 'x';
              serialTxStruct.arg1 = pin;
              serialTxStruct.arg2 = val;
              serialTxStruct.status = 200;   //Successful
              break;
            }
            else{
              break;    //failed
            }
        case PIN_MODE:
        /* c <IDE_PIN_NUM> <0:INPUT, 1: OUTPUT> */
            if (types(serialRxStruct.arg1) == "float" && types(serialRxStruct.arg2) == "float"
            && (serialRxStruct.arg1 - floor(serialRxStruct.arg1)) == 0 && (serialRxStruct.arg2 - floor(serialRxStruct.arg2)) == 0){
              int pin = int(serialRxStruct.arg1);
              int mode = int(serialRxStruct.arg2);

              if (mode == 0) pinMode(pin, INPUT);
              else if (mode == 1) pinMode(pin, OUTPUT);
              
              serialTxStruct.cmd = 'c';
              serialTxStruct.arg1 = pin;
              serialTxStruct.arg2 = mode;
              serialTxStruct.status = 200;   //Successful
              break; 
            }
            else{
              break;    //Failed
            }
        case PING: // WIP
        /* p <IDE_PIN_NUM>  */
            //Serial.println(Ping(arg1));
            serialTxStruct.status = 500;   //Not Implemented
            break;
        case SERVO_WRITE: //WIP
        /* s <IDE_PIN_NUM> <TARGET_SERVO_POSITION> */
            // servos[arg1].setTargetPosition(arg2);
            // Serial.println("OK")
            serialTxStruct.status = 500;   //Not Implemented
            break;
        case SERVO_READ: //WIP
        /* t <IDE_PIN_NUM> */
            // Serial.println(servos[arg1].getServo().read());
            serialTxStruct.status = 500;   //Not Implemented
            break;
        case READ_ENCODERS:
        /* e */
        // Send the counts and timestamp (microseconds) over serial
            serialTxStruct.arg1 = -readEncoder(LEFT);
            serialTxStruct.arg2 = readEncoder(RIGHT);
            serialTxStruct.arg3 = micros();

            resetEncoder(LEFT);
            resetEncoder(RIGHT);

            serialTxStruct.cmd = 'e';
            serialTxStruct.status = 200;  //Successful
            break;
        case RESET_MOTORS:
        /* r */
            resetEncoder(LEFT);
            resetEncoder(RIGHT);
            resetPID(LEFT);
            resetPID(RIGHT);
            setMotorSpeeds(0,0);
            RX_ALLOWED = true;
            TX_ALLOWED = true;
            
            serialTxStruct.cmd = 'r';
            serialTxStruct.status = 200;  //Successful
            break;
        case MOTOR_CONTROL:
        /* m <distance (metres)> <relative heading (degrees)> */
        // Sets motors' closed-loop distance, based on input
            /* Reset the auto stop timer */
            lastMotorCommand = micros();
            /* 0 input */
            if (serialRxStruct.arg1  == 0 && serialRxStruct.arg2 == 0) {
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

            /* Calculates encoder counts for straightline movement */
            float straight_counts = (serialRxStruct.arg1/(2*PI*WHEEL_RADIUS)) * 48 * SPINDLE_TO_ENCODER_GEAR_RATIO;  // Distance in m
            /* Calculates encoder counts for turning movement */
            float turning_counts = (serialRxStruct.arg2*(48*SPINDLE_TO_ENCODER_GEAR_RATIO)*(WHEEL_BASE/2))/(360*WHEEL_RADIUS);  // Angle in degrees

            leftPID.Straight_CountsToTarget = floor(straight_counts);
            rightPID.Straight_CountsToTarget = floor(straight_counts);
            leftPID.Turning_CountsToTarget = floor(-turning_counts);
            rightPID.Turning_CountsToTarget = floor(turning_counts);

            serialTxStruct.cmd = 'm';
            serialTxStruct.arg1 = float(straight_counts);
            serialTxStruct.arg2 = float(turning_counts);  
            serialTxStruct.arg3 = 0.0;
            serialTxStruct.arg4 = 0.0;
            serialTxStruct.status = 202;    //Accepted
            break;
        case MOTOR_RAW_PWM:
            /* o <L_PWM> <R_PWM> */
            /* Reset the auto stop timer */
            lastMotorCommand = micros();
            resetPID(LEFT);
            resetPID(RIGHT);
            moving = 0; // Sneaky way to temporarily disable the PID
            setMotorSpeeds(int(serialRxStruct.arg1), int(serialRxStruct.arg2));
            
            serialTxStruct.cmd = 'o';
            serialTxStruct.arg1 = int(serialRxStruct.arg1);
            serialTxStruct.arg2 = int(serialRxStruct.arg2);
            serialTxStruct.status = 200;  //Successful
            break; 
        case UPDATE_PID_L_STRAIGHT:
            /* y <L_STRAIGHT_KP> <L_STRAIGHT_KI> <L_STRAIGHT_KD> <L_STRAIGHT_KO> */
            /* Reset the auto stop timer */
            L_Kp = serialRxStruct.arg1;
            L_Ki = serialRxStruct.arg2;
            L_Kd = serialRxStruct.arg3;
            L_Ko = serialRxStruct.arg4;

            serialTxStruct.cmd = 'y';
            serialTxStruct.arg1 = L_Kp;
            serialTxStruct.arg2 = L_Ki;
            serialTxStruct.arg3 = L_Kd;
            serialTxStruct.arg4 = L_Ko;
            serialTxStruct.status = 200;  //Successful
            break;
        case UPDATE_PID_R_STRAIGHT:
            /* u <L_STRAIGHT_KP> <L_STRAIGHT_KI> <L_STRAIGHT_KD> <L_STRAIGHT_KO> */
            /* Reset the auto stop timer */
            R_Kp = serialRxStruct.arg1;
            R_Ki = serialRxStruct.arg2;
            R_Kd = serialRxStruct.arg3;
            R_Ko = serialRxStruct.arg4;

            serialTxStruct.cmd = 'u';
            serialTxStruct.arg1 = R_Kp;
            serialTxStruct.arg2 = R_Ki;
            serialTxStruct.arg3 = R_Kd;
            serialTxStruct.arg4 = R_Ko;
            serialTxStruct.status = 200;  //Successful
            break;
        case UPDATE_PID_L_TURNING:
            /* 6 <L_STRAIGHT_KP> <L_STRAIGHT_KI> <L_STRAIGHT_KD> <L_STRAIGHT_KO> */
            /* Reset the auto stop timer */
            Lt_Kp = serialRxStruct.arg1;
            Lt_Ki = serialRxStruct.arg2;
            Lt_Kd = serialRxStruct.arg3;
            Lt_Ko = serialRxStruct.arg4;

            serialTxStruct.cmd = '6';
            serialTxStruct.arg1 = Lt_Kp;
            serialTxStruct.arg2 = Lt_Ki;
            serialTxStruct.arg3 = Lt_Kd;
            serialTxStruct.arg4 = Lt_Ko;
            serialTxStruct.status = 200;  //Successful
            break;
        case UPDATE_PID_R_TURNING:
            /* 7 <L_STRAIGHT_KP> <L_STRAIGHT_KI> <L_STRAIGHT_KD> <L_STRAIGHT_KO> */
            /* Reset the auto stop timer */
            Rt_Kp = serialRxStruct.arg1;
            Rt_Ki = serialRxStruct.arg2;
            Rt_Kd = serialRxStruct.arg3;
            Rt_Ko = serialRxStruct.arg4;

            serialTxStruct.cmd = '7';
            serialTxStruct.arg1 = Rt_Kp;
            serialTxStruct.arg2 = Rt_Ki;
            serialTxStruct.arg3 = Rt_Kd;
            serialTxStruct.arg4 = Rt_Ko;
            serialTxStruct.status = 200;  //Successful
            break;
          case GET_PID_STRAIGHT_L:
            serialTxStruct.cmd = '1';
            serialTxStruct.arg1 = L_Kp;
            serialTxStruct.arg2 = L_Ki;
            serialTxStruct.arg3 = L_Kd;
            serialTxStruct.arg4 = L_Ko;
            serialTxStruct.status = 200;  //Successful
            break;          
          case GET_PID_TURNING_L:
            serialTxStruct.cmd = '2';
            serialTxStruct.arg1 = Lt_Kp;
            serialTxStruct.arg2 = Lt_Ki;
            serialTxStruct.arg3 = Lt_Kd;
            serialTxStruct.arg4 = Lt_Ko;
            serialTxStruct.status = 200;  //Successful
            break;          
          case GET_PID_STRAIGHT_R:
            serialTxStruct.cmd = '3';
            serialTxStruct.arg1 = R_Kp;
            serialTxStruct.arg2 = R_Ki;
            serialTxStruct.arg3 = R_Kd;
            serialTxStruct.arg4 = R_Ko;
            serialTxStruct.status = 200;  //Successful
            break;          
          case GET_PID_TURNING_R:
            serialTxStruct.cmd = '4';
            serialTxStruct.arg1 = Rt_Kp;
            serialTxStruct.arg2 = Rt_Ki;
            serialTxStruct.arg3 = Rt_Kd;
            serialTxStruct.arg4 = Rt_Ko;
            serialTxStruct.status = 200;  //Successful
            break;
        default:
            serialTxStruct.status = 404;  //Wrong command
            break;
        
    }
    TX_REQUESTED = true;
    return;
}


/* Setup function--runs once at startup. */
void setup() {
    Serial.begin(BAUD_RATE);
    serialInterface.begin(Serial);
    initMotorController();
    resetPID(LEFT);
    resetPID(RIGHT);

    // for debugging
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop(){
    /* Read from Rx buffer if data available, and prepare Tx data*/
    if (RX_ALLOWED && serialInterface.available()){
        serialInterface.rxObj(serialRxStruct);
        executeSerialCmd();
    }

    /* Send datum if needed */
    if (TX_ALLOWED && TX_REQUESTED){
      RX_ALLOWED = false;
      serialInterface.sendDatum(serialTxStruct);
      TX_REQUESTED = false;
      RX_ALLOWED = true;
    }
    digitalWrite(LED_BUILTIN, LOW);
    // run a PID calculation at the appropriate intervals
    if (micros() > nextPID) {
        /* Blocks further Serial commands from interfering with the PID loop */
        RX_ALLOWED = false;
        updatePID();
        nextPID += PID_INTERVAL;
    }
    
    // Check to see if we have exceeded the auto-stop interval
    if (((micros() - lastMotorCommand) > AUTO_STOP_INTERVAL)) {
          setMotorSpeeds(0, 0);
          moving = 0;
    }


    //int i;
    //for (i = 0; i < N_SERVOS; i++) {
    //    servos[i].doSweep();
    //}
}


