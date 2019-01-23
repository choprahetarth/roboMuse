#include <Sabertooth.h>   // Header file for Sabertooth Motor Driver
#include <Encoder.h>      // Header file for the Encoders
#include <PID_v1.h>       // Header file for PID controller

// Flags
String modeSelected;
String motionType;

// Sabertooth arguments
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

// Odometry Variables
double oldLeftEncoderValue=0, newLeftEncoderValue=0;
double oldRightEncoderValue=0, newRightEncoderValue=0;
double leftEncoderIncrement=0, rightEncoderIncrement=0;
float conversionFactor = 0.1;   // rotary to linear
double leftWheelIncrement=0, rightWheelIncrement=0;
double width = 515;   // distance between wheels
float theta = 0;
float feedbackVariable = 0;    // Error value

// PID Variables
double minVal=-15, maxVal=15;
double input=0, setpoint=0; // Input_Error_value, Controler_Output_value, Desired_Error_value
double KpL = 24, KiL = 2.2, KdL = 0, outputL=0;   // Proportional, Integral & Derivative coefficients
double KpR = 24, KiR = 2.2, KdR = 0, outputR=0;    // of respective motors for PID control

Sabertooth saberTooth(128, Serial2);  // Packetized serial mode, Non-LI, 128 bit Addr. (0,0,1,1,1,1)
Encoder enCoder_1(20,21); // Left hand side enc., +ve value means forward
Encoder enCoder_2(2,3);   // Right hand side enc., -ve value means forward
PID PID_L(&input, &outputL, &setpoint, KpL, KiL, KdL, DIRECT); // Direct mode : Increase output to  increase input
PID PID_R(&input, &outputR, &setpoint, KpR, KiR, KdR, DIRECT);

void odometryCalc(){
  newLeftEncoderValue = double(enCoder_1.read());
  newRightEncoderValue = double(enCoder_2.read());
  leftEncoderIncrement = newLeftEncoderValue - oldLeftEncoderValue;
  rightEncoderIncrement = newRightEncoderValue - oldRightEncoderValue;
  Serial.print("Left wheel : ");
  Serial.println(leftEncoderIncrement);
  Serial.print("Right wheel : ");
  Serial.println(rightEncoderIncrement);
  leftWheelIncrement = leftEncoderIncrement*conversionFactor; // left side advanced-by-distance
  rightWheelIncrement = rightEncoderIncrement*conversionFactor*(-1); // right side advanced-by-distance
  theta = atan((rightWheelIncrement-leftWheelIncrement)/width);
  Serial.print("Theta : ");
  Serial.println(theta);
  feedbackVariable = theta;
  oldLeftEncoderValue = newLeftEncoderValue;
  oldRightEncoderValue = newRightEncoderValue;
}

void resetCoordinates(){
  oldLeftEncoderValue=0; oldRightEncoderValue=0;
  newLeftEncoderValue=0; newRightEncoderValue=0;
  leftEncoderIncrement=0; rightEncoderIncrement=0;
  leftWheelIncrement=0; rightWheelIncrement=0;
  enCoder_1.write(0);
  enCoder_2.write(0);
}


void setup() {
  Serial.begin(9600);   // Serial communication with rasPi
  Serial2.begin(9600);  // Serial communication with Sabertooth motor driver, default baud rate

  PID_L.SetOutputLimits(minVal,maxVal);   // [Min,Max] values of output
  PID_L.SetMode(AUTOMATIC);  // Automatic = ON, Manual = OFF
  PID_R.SetOutputLimits(minVal,maxVal);
  PID_R.SetMode(AUTOMATIC);

  resetCoordinates();
}

void loop() {
  Serial.println("Please choose the mode of operation -");
  Serial.println("\t m = Manual,\n\t a = Autonomous,\n\t n = Navigation.");
  while(!Serial.available()){
    // Wait.
  }
  modeSelected = Serial.readString();
  if(modeSelected == "m"){
    Serial.println("Mode Selected : Manual !");
    manualMode();
  }
  else if(modeSelected == "a"){
    Serial.println("Mode Selected : Autonomous !");
    autonomousMode();
  }
  else if(modeSelected == "n"){
    Serial.println("Mode Selected : Navigation !");
    navigationMode();
  }
  else{
    Serial.println("Invalid operand !!\n");
  }
}
