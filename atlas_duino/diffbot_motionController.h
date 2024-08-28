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
// PID params Setters - Straightline
double L_Kp = 0.1;
double L_Ki = 0.05;
double L_Kd = 5;
double L_Ko = 10;
double R_Kp = 0.1;
double R_Ki = 0.05;
double R_Kd = 5;
double R_Ko = 10;
// PID params Setters - Straightline
double Lt_Kp = 0.1;
double Lt_Ki = 0;
double Lt_Kd = 5;
double Lt_Ko = 10;
double Rt_Kp = 0.1;
double Rt_Ki = 0;
double Rt_Kd = 5;
double Rt_Ko = 10;
// Runtime PID variables
double Kp = 0;
double Ki = 0;
double Kd = 0;
double Ko = 0;

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
void resetPID(int motor){

  switch(motor){
    case LEFT:
        leftPID.Straight_CountsToTarget = 0.0;
        leftPID.Turning_CountsToTarget = 0.0;
        leftPID.Encoder = readEncoder(LEFT);
        leftPID.output = 0;
        leftPID.PrevErr = 0;
        leftPID.ITerm = 0;
        resetEncoder(LEFT);
        break;
    case RIGHT:
        rightPID.Straight_CountsToTarget = 0.0;
        rightPID.Turning_CountsToTarget = 0.0;
        rightPID.Encoder = readEncoder(RIGHT);
        rightPID.output = 0;
        rightPID.PrevErr = 0;
        rightPID.ITerm = 0;
        resetEncoder(RIGHT);
        break;
  }
 
  leftPID.motor = LEFT;
  rightPID.motor = RIGHT;

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

  if (abs(Perror) < PID_ERROR_MARGIN){
    //p->PrevErr = 0;

    if (turning){  // Deactivate turning mode after passing threshold
      switch(p->motor){
        case RIGHT:
          //Serial.print("Right wheel. Turning Complete.");
          //Serial.print(",R:");
          //Serial.print(rightPID.PrevErr);
          //Serial.print(",R_e:");
          //Serial.println(rightPID.output*10);
          p->output = 0;
          rightPID.Turning_CountsToTarget = 0.0;
          rightPID.Encoder = readEncoder(RIGHT);
          rightPID.output = 0;
          rightPID.PrevErr = 0;
          rightPID.ITerm = 0;
          resetEncoder(RIGHT);        
        
        case LEFT:
          //Serial.print("Left wheel. Turning Complete.");
          //Serial.print(",L:");
          //Serial.print(leftPID.PrevErr);
          //Serial.print(",L_e:");
          //Serial.println(leftPID.output*10);
          p->output = 0;  
          leftPID.Turning_CountsToTarget = 0.0;
          leftPID.Encoder = readEncoder(LEFT);
          leftPID.output = 0;
          leftPID.PrevErr = 0;
          leftPID.ITerm = 0;
          resetEncoder(LEFT);
      }
      if (){
        turning = 0;
        delay(100);
      }
    }
    else{
      switch(p->motor){
        case RIGHT:
          p->output = 0;        
          resetPID(RIGHT);
        case LEFT:
          p->output = 0;  
          resetPID(LEFT);  
      }
      
      if (leftPID.Turning_CountsToTarget == 0 && rightPID.Turning_CountsToTarget == 0 && leftPID.Straight_CountsToTarget == 0 && rightPID.Straight_CountsToTarget == 0){
        moving = 0;
        Serial.println("OK.MOTION_CONTROLLER.Movement Complete");
      }
    }
    return;
  }

  if (turning){
    switch (p->motor){
      case LEFT:
          Kp = Lt_Kp;
          Ki = Lt_Ki;
          Kd = Lt_Kd;
          Ko = Lt_Ko;
          break;
        case RIGHT:
          Kp = Rt_Kp;
          Ki = Rt_Ki;
          Kd = Rt_Kd;
          Ko = Rt_Ko;
          break;
    } 
  }
  else{
    switch (p->motor){
      case LEFT:
          Kp = L_Kp;
          Ki = L_Ki;
          Kd = L_Kd;
          Ko = L_Ko;
          break;
        case RIGHT:
          Kp = R_Kp;
          Ki = R_Ki;
          Kd = R_Kd;
          Ko = R_Ko;
          break;
    }
  }
  

  output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->ITerm) / Ko;
  p->PrevErr = Perror;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else if (output > 0 && output <= MIN_PWM && output >= MIN_PWM-MIN_PWM_ALLOW_ZONE)
    output = MIN_PWM;
  else if (output < 0 && output >= -MIN_PWM && output <= -MIN_PWM+MIN_PWM_ALLOW_ZONE)
    output = -MIN_PWM;
  //Straightline motion
  else if (!turning && output >= -MIN_PWM+MIN_PWM_ALLOW_ZONE && output <= MIN_PWM-MIN_PWM_ALLOW_ZONE)
    output = 0;
  //Turning motion
  else if (turning && output >= -MIN_PWM && output < 0)
    output = -MIN_PWM-40 ;
  else if (turning && output <= MIN_PWM && output > 0)
    output = MIN_PWM+40 ;
  else p->ITerm += Ki * Perror;

  p->output = output;

  /* Serial Plotter */
  //Serial.print("Fixed:");
  //Serial.print(50000);
  //Serial.print(",L:");
  //Serial.print(leftPID.PrevErr);
  //Serial.print(",R:");
  //Serial.print(rightPID.PrevErr);
  //Serial.print(",L_e:");
  //Serial.print(leftPID.output*10);
  //Serial.print(",R_e:");
  //Serial.println(rightPID.output*10);
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
    if (leftPID.PrevErr != 0 || rightPID.PrevErr != 0){
      resetPID(LEFT);
      resetPID(RIGHT);
    } 
    return;
  }

  /* Compute PID update for each motor */
  doPID(&rightPID);
  doPID(&leftPID);

  /* Set the motor speeds accordingly */
  setMotorSpeeds(leftPID.output, rightPID.output);
}