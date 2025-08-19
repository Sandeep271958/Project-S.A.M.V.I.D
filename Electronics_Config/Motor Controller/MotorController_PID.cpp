//Pins for Motor 1
#define RH_ENCODER_A 2 
#define RH_ENCODER_B 8
#define RH_PWM 6
#define RH_DIR 7

//Pins for Motor 2
#define LH_ENCODER_A 3
#define LH_ENCODER_B 9
#define LH_PWM 5
#define LH_DIR 4

#define GR 71.2 // Gear Ratio
#define PPR 7.0 // Pulse Per Revolution

#define MIN_PWM 0
#define MAX_PWM 255
// #define MIN_SPD 0.1 // in revolutions per second
// #define MAX_SPD 1.6 // in revolutions per second
#define MIN_SPD 0 // in revolutions per second
#define MAX_SPD 1.67 // in revolutions per second

// Proportional speed controller
#define left_motor_P_gain 0.6
#define right_motor_P_gain 0.6 

// Integral speed controller
#define left_motor_I_gain 0.0 
#define right_motor_I_gain 0.0 

// Derivative speed controller
#define left_motor_D_gain 0.01
#define right_motor_D_gain 0.01

#define W_RADIUS 0.11 //m
#define W_DIST 0.21 //m
#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif

//For serial communication
char receivedChars[12];
boolean newData = false;
float vL, vR;
//For time measurment
unsigned long currTime = 0, prevTime = 0; // time measurment
double dt = 0.0;                          //deltatime in seconds

//keeping it safe
long encoder_minimum = -2147483640;
long encoder_maximum = 2147483640;

//Feed Forward Control Coefficient(Linear Model)
float ff = (MAX_PWM-MIN_PWM)/(MAX_SPD - MIN_SPD);
//float ff = 1;
//'Motor' Class for specific data corresponding to each motor
class Motor
{

public:
  float desiredSpeed = 0; // in revolutions per second
  float currentSpeed = 0; // in revolutions per second
  long encoderCount = 0;
  long prevEncoderCount = 0;
  int output;
  float Kp = 0.0;
  float Kd = 0.0;
  float Ki = 0.0;
  long prevError= 0;
  long currentError = 0;
  long errorSum= 0;

  void setDesiredSpeed(float s)
  // s is desired speed in revolutions per second
  {
    if (abs(s) > MAX_SPD)
    {
      int signofspeed = (s > 0) - (s < 0);
      desiredSpeed = signofspeed*MAX_SPD;
      return;
    }

    if (abs(s) < MIN_SPD)
    {
      desiredSpeed = 0;
      return;
    }
    desiredSpeed = s;
   
  }

  void computeOutput(float dt)
  //dt is deltatime in seconds
  {
    currentSpeed = ((encoderCount - prevEncoderCount) / (PPR * dt * GR)); //in revolutions per second
    prevEncoderCount = encoderCount;
    currentError = desiredSpeed - currentSpeed;
    errorSum += currentError*dt;
    output = ff*desiredSpeed + Kp*(desiredSpeed - currentSpeed) + Kd*(currentError-prevError)/dt + Ki*errorSum;//Feed Forward + PID Controller
    prevError = currentError;
    Serial.print(output);
  }

  Motor(float KpGain, float KdGain,float KiGain)
  //Constructor for Motor Class
  {
    Kp = KpGain;
    Kd = KdGain;
    Ki = KiGain;
  }
};

Motor leftMotor(left_motor_P_gain,left_motor_D_gain,left_motor_I_gain);
Motor rightMotor(right_motor_P_gain,right_motor_D_gain,right_motor_I_gain);

// Checking if the code works without any PID gains
// Motor leftMotor(0,0,0);
// Motor rightMotor(0,0,0);

void setup()
{
  leftMotor.setDesiredSpeed(0.0);
  rightMotor.setDesiredSpeed(0.0);

  pinMode(LH_ENCODER_A, INPUT);
  pinMode(LH_ENCODER_B, INPUT);
  pinMode(RH_ENCODER_A, INPUT);
  pinMode(RH_ENCODER_B, INPUT);

  pinMode(RH_DIR, OUTPUT);
  pinMode(LH_DIR, OUTPUT);
  pinMode(RH_PWM, OUTPUT);
  pinMode(LH_PWM, OUTPUT);


  // initialize hardware interrupts for FALL event
  attachInterrupt(digitalPinToInterrupt(LH_ENCODER_A), leftEncoderFallEvent, FALLING);
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), rightEncoderFallEvent, FALLING);

  Serial.begin(115200);

  Serial.println("<Arduino is ready>");
}

void loop()
{
  //Get commanded speed
  recvWithStartEndMarkers();
  if (newData == true)
  {
    parseData();
    newData = false;
  }

  //Compute delta time
  currTime = micros();
  dt = (currTime - prevTime) / (1000000.0); // in seconds
  prevTime = currTime;

  //Run Proportional speed controller and get output
  rightMotor.computeOutput(dt);
  leftMotor.computeOutput(dt);

  //Apply control signal on the motors
  if (rightMotor.output > 0)
  {
    digitalWrite(RH_DIR, 1);
  }
  else
  {
    digitalWrite(RH_DIR, 0);
  }
  
  if (leftMotor.output > 0)
  {
    digitalWrite(LH_DIR, 1);
  }
  else
  {
    digitalWrite(LH_DIR, 0);
  }

  if (abs(rightMotor.output) > 255)
  {
    rightMotor.output = 255;
  }
  if ((abs(rightMotor.output)) < MIN_PWM)
  {
    rightMotor.output = 0;
  }
  
  if (abs(leftMotor.output) > 255)
  {
    leftMotor.output = 255;
  }
  if (abs(leftMotor.output) < MIN_PWM)
  {
    leftMotor.output = 0;
  }
  analogWrite(LH_PWM, abs(leftMotor.output));
  analogWrite(RH_PWM, abs(rightMotor.output));

  //return current tick count
  /*Serial.print("<");
  Serial.print(leftMotor.encoderCount);
  Serial.print(",");
  Serial.print(rightMotor.encoderCount);
  Serial.print(">\n");*/
  delay(10);
  /*Serial.print("<");
  Serial.print(23);
  Serial.print(",");
  Serial.print(4);
  Serial.print(">\n");*/
  
  //Debug Code
  /*Serial.print("<");
  Serial.print(round(leftMotor.encoderCount));
  Serial.print(",");
  Serial.print(round(rightMotor.encoderCount));
  Serial.print(",");
  Serial.print(leftMotor.currentSpeed);
  Serial.print(",");
  Serial.print(rightMotor.currentSpeed);
  Serial.print(",");
  Serial.print(leftMotor.output);
  Serial.print(",");
  Serial.print(rightMotor.output);
  Serial.print(",");
  Serial.print(rightMotor.desiredSpeed);
  Serial.print(",");
  Serial.print(leftMotor.desiredSpeed);
  Serial.print(">\n");*/
  Serial.print("<");
  Serial.print(leftMotor.currentSpeed);
  Serial.print(",");
  Serial.print(rightMotor.currentSpeed);
  Serial.print(",");
  Serial.print(">\t");
  Serial.print("<");
  Serial.print(rightMotor.desiredSpeed);
  Serial.print(",");
  Serial.print(leftMotor.desiredSpeed);
  Serial.print(">\n");
}

void leftEncoderFallEvent()
{ 
  
  if (digitalRead(LH_ENCODER_B) == LOW)
  { 

   
    if (leftMotor.encoderCount < encoder_maximum)
      leftMotor.encoderCount++;
  }
  else
  { 
    if (leftMotor.encoderCount > encoder_minimum)
      leftMotor.encoderCount--;
  }
}


void rightEncoderFallEvent()
{ 
  
  if (digitalRead(RH_ENCODER_B) == LOW)
  {
    if (rightMotor.encoderCount < encoder_maximum)
      rightMotor.encoderCount--;
  }
  else
  {
    if (rightMotor.encoderCount > encoder_minimum)
      rightMotor.encoderCount++;
  }
}

// The data parsing side 
// Data expected in format<fb speed, speed for rotation> unit being rps*100

//============
void recvWithStartEndMarkers()
{
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false)
  {
    rc = Serial.read();

    if (recvInProgress == true)
    {
      if (rc != endMarker)
      {
        receivedChars[ndx] = rc;
        ndx++;
      }
      else
      {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker)
    {
      recvInProgress = true;
    }
  }
}

//============

void parseData()
// split the data into its parts
{
  char *strtokIndx; // this is used by strtok() as an index
  float fb, rot;
  float wL, wR;
  strtokIndx = strtok(receivedChars, ","); // get the first part - left speed
  fb = atof(strtokIndx); //front-back speed (m/s)
  strtokIndx = strtok(NULL, ","); // get the second part - right speed
  rot = atof(strtokIndx);
  wL = (-fb -(rot*W_DIST))/(W_RADIUS*2*M_PI);
  wR = (-fb +(rot*W_DIST))/(W_RADIUS*2*M_PI);
  leftMotor.setDesiredSpeed(wL);
  rightMotor.setDesiredSpeed(wR);

  // This is used if vL and vR are given directly
  //leftMotor.setDesiredSpeed(atof(strtokIndx)/100);
  //rightMotor.setDesiredSpeed(atof(strtokIndx)/100);
}
