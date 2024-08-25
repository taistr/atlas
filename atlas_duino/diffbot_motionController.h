/* Functions and type-defs for PID control.

   Taken mostly from Mike Ferguson's ArbotiX code which lives at:
   
   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/

/* PID setpoint info For a Motor */
typedef struct {
  long Straight_CountsToTarget;    // Straight line counts to target
  long Turning_CountsToTarget;    //  Counts to desired heading
  long Encoder;                // encoder count
  int motor;

  /*
  * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  */
  //int PrevInput;                // last input
  int PrevErr;                   // last error

  /*
  * Using integrated term (ITerm) instead of integrated error (Ierror),
  * to allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //int Ierror;
  double ITerm;                    //integrated term

  long output;                    // last motor setting
}
SetPointInfo;

SetPointInfo leftPID, rightPID;


/* PID Parameters */
double Kp = 2;
double Ki = 1;
double Kd = 0;
double Ko = 100;

unsigned char moving = 0; // is the base in motion?
unsigned char turning = 1; // is the base turning?


/*
* Initialize PID variables to zero to prevent startup spikes
* when turning PID on to start moving
* In particular, assign both Encoder and PrevEnc the current encoder value
* See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
* Note that the assumption here is that PID is only turned on
* when going from stop to moving, that's why we can init everything on zero.
*/
void resetPID(){
  leftPID.Straight_CountsToTarget = 0.0;
  leftPID.Turning_CountsToTarget = 0.0;
  leftPID.Encoder = readEncoder(LEFT);
  leftPID.output = 0;
  leftPID.PrevErr = 0;
  leftPID.ITerm = 0;

  rightPID.Straight_CountsToTarget = 0.0;
  rightPID.Turning_CountsToTarget = 0.0;
  rightPID.Encoder = readEncoder(RIGHT);
  rightPID.output = 0;
  rightPID.PrevErr = 0;
  rightPID.ITerm = 0;
  
  moving = 0;

  leftPID.motor = LEFT;
  rightPID.motor = RIGHT;

  resetEncoder(LEFT);
  resetEncoder(RIGHT);

  //Serial.println("PID Reset.");
}

/* PID routine to compute the next motor commands */
void doPID(SetPointInfo * p) {
  long Perror;
  long output;

  if (!turning){  // Select set point based on mode
    Perror = p->Straight_CountsToTarget;
  }
  else{
    Perror = p->Turning_CountsToTarget;
    //Serial.println("Turning");
  }

  if (abs(Perror) < 100){
    p->PrevErr = 0;

    if (turning){  // Deactivate turning mode after passing threshold
      turning = 0;
      //Serial.println("Turning Complete.");
      if (p->motor == RIGHT){
        p->output = 0;
        rightPID.Turning_CountsToTarget = 0;
        rightPID.PrevErr = 0;
        resetEncoder(RIGHT);
      }
      else{
        p->output = 0;        
        leftPID.Turning_CountsToTarget = 0;
        leftPID.PrevErr = 0;
        resetEncoder(LEFT);
      }
      delay(100);
    }
    else{
      if (p->motor == RIGHT){
        p->output = 0;        
        rightPID.Straight_CountsToTarget = 0;
        rightPID.PrevErr = 0;
        resetEncoder(RIGHT);
      }
      else{
        p->output = 0;        
        leftPID.Straight_CountsToTarget = 0;
        leftPID.PrevErr = 0;
        resetEncoder(LEFT);
      }
      delay(100);
      //Serial.println("Straight Line Complete.");
      //resetPID();
    }
    Serial.print("Fixed:");
    Serial.print(50000);
    Serial.print(",L:");
    Serial.print(leftPID.PrevErr);
    Serial.print(",R:");
    Serial.print(rightPID.PrevErr);
    Serial.print(",L_e:");
    Serial.print(leftPID.output*10);
    Serial.print(",R_e:");
    Serial.println(rightPID.output*10);
    return;
  }

  /*
  * Avoid derivative kick and allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->ITerm) / Ko;
  p->PrevErr = Perror;
  //output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
  //p->PrevEnc = p->Encoder;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
  /*
  * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
    p->ITerm += Ki * Perror;

  p->output = output;
  Serial.print("Fixed:");
  Serial.print(50000);
  Serial.print(",L:");
  Serial.print(leftPID.PrevErr);
  Serial.print(",R:");
  Serial.print(rightPID.PrevErr);
  Serial.print(",L_e:");
  Serial.print(leftPID.output*10);
  Serial.print(",R_e:");
  Serial.println(rightPID.output*10);
}

/* Read the encoder values and call the PID routine */
void updatePID() {

  /* Read the encoders */
  if (!turning){
    leftPID.Straight_CountsToTarget = leftPID.Straight_CountsToTarget + readEncoder(LEFT);
    rightPID.Straight_CountsToTarget = rightPID.Straight_CountsToTarget - readEncoder(RIGHT);
  }
  else{
    leftPID.Turning_CountsToTarget = leftPID.Turning_CountsToTarget + readEncoder(LEFT);
    rightPID.Turning_CountsToTarget = rightPID.Turning_CountsToTarget - readEncoder(RIGHT);
  }

  resetEncoder(LEFT);
  resetEncoder(RIGHT);
  
  /* If we're not moving there is nothing more to do */
  if (!moving){
    /*
    * Reset PIDs once, to prevent startup spikes,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
    * PrevInput is considered a good proxy to detect
    * whether reset has already happened
    */
    if (leftPID.PrevErr != 0 || rightPID.PrevErr != 0) resetPID();
    return;
  }

  /* Compute PID update for each motor */
  doPID(&rightPID);
  doPID(&leftPID);

  /* Set the motor speeds accordingly */
  setMotorSpeeds(leftPID.output, rightPID.output);
}