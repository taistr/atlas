#include "duino_cmds.h"
#include "encoder_interface.h"
#include <Servo.h>

/* Maximum PWM signal */
#define MAX_PWM        255

// Serial
#define BAUD_RATE 115200

// Encoder
#define ENC_TIME_DELTA_THRESHOLD 100000

/* Run the PID loop at 30 times per second */
#define PID_RATE    30     // Hz

/* Convert the rate into an interval */
const int PID_INTERVAL = 1000*1000 / PID_RATE;

/* Track the next time we make a PID calculation */
unsigned long nextPID = PID_INTERVAL;

/* Stop the robot if it hasn't received a movement command
in this number of microseconds */
#define AUTO_STOP_INTERVAL 2000000
long lastMotorCommand = AUTO_STOP_INTERVAL;

// Initialising variables
//Variables to help parse serial commands
int arg = 0;
int idx = 0;

//Variable to hold serial command character
char cmd;

//Variable to hold serial data
char chr;

//Variables to hold arguments
char argv1[16];
char argv2[16];

//Variables to hold arguments integer codes
long arg1;
long arg2;

void resetCmd(){
    cmd = NULL;
    memset(argv1, 0, sizeof(argv1));
    memset(argv2, 0, sizeof(argv1));
    arg1 = 0;
    arg2 = 0;
    arg = 0;
    idx = 0;
}

int executeSerialCmds(){
    int i = 0;
    char *p = argv1;
    char *str;
    int pid_args[4];
    arg1 = atoi(argv1);
    arg2 = atoi(argv2);
    
    switch(cmd){
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
            sprintf(data, "Analog write to pin %3d with value %3d ",arg1, arg2);
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
            // Send the counts and timestamp (microseconds) over serial
            Serial.print("L:");
            Serial.print(-readEncoder(LEFT));
            Serial.print(",R:");
            Serial.print(readEncoder(RIGHT));
            Serial.print(",Timestamp (microseconds):");
            Serial.println(micros());  // Send time delta in microseconds
            resetEncoder(LEFT);
            resetEncoder(RIGHT);
            break;
        case RESET_ENCODERS:
            resetEncoder(LEFT);
            resetEncoder(RIGHT);
            //resetPID();
            Serial.println("Encoders & PID Reset");
            break;
        case MOTOR_SPEEDS:
            /* Reset the auto stop timer */
            // lastMotorCommand = micros();
            // if (arg1 == 0 && arg2 == 0) {
            // setMotorSpeeds(0, 0);
            // resetPID();
            // moving = 0;
            // }
            // else moving = 1;
            // leftPID.TargetTicksPerFrame = arg1;
            // rightPID.TargetTicksPerFrame = arg2;
            // Serial.println("OK"); 
            break;
        case MOTOR_RAW_PWM:
            /* Reset the auto stop timer */
            // lastMotorCommand = micros();
            // resetPID();
            // moving = 0; // Sneaky way to temporarily disable the PID
            // setMotorSpeeds(arg1, arg2);
            // Serial.println("OK"); 
            break;
        case UPDATE_PID:
            // while ((str = strtok_r(p, ":", &p)) != '\0') {
            // pid_args[i] = atoi(str);
            // i++;
            // }
            // Kp = pid_args[0];
            // Kd = pid_args[1];
            // Ki = pid_args[2];
            // Ko = pid_args[3];
            // Serial.println("OK");
            break;
        default:
            Serial.println("Invalid Command");
            break;
        
    }
}


/* Setup function--runs once at startup. */
void setup() {
    Serial.begin(BAUD_RATE);
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
        }
    }
  
    // run a PID calculation at the appropriate intervals
    //if (micros() > nextPID) {
    //    updatePID();
    //    nextPID += PID_INTERVAL;
    //}
    
    // Check to see if we have exceeded the auto-stop interval
    //if ((micros() - lastMotorCommand) > AUTO_STOP_INTERVAL) {;
    //    setMotorSpeeds(0, 0);
    //    moving = 0;
    //}


    //int i;
    //for (i = 0; i < N_SERVOS; i++) {
    //    servos[i].doSweep();
    //}
}