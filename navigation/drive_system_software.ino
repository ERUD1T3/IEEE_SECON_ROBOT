//////////////////////////////////////
//IEEE Robot 2018: Drive System
/////////////////////////////////////

#include <Adafruit_MotorShield.h> //Motor shield v2 
//#include "utility/Adafruit_MS_PWMServoDriver.h"	//In case Servo is used 
#include <Wire.h>
#include <math.h>
#define MAX_SPEED 1200								//1200 RPM for Bipolar Stepper
#define ROTATE_ANGLE 0.6159936776 //
#define ANGLE_CORRECTION_FACTOR 5  //
#define HALF_ROBOT_LENGTH 6							//refers to the length of the robot from center to edge
#define SAFETY_SENSOR_DISTANCE 1					//maintain a safety distance of at least one inch before activating sensor for check


// functions have to be adjusted if the measurment is in inches or centimeter
#define WHEEL_DIAMETER								// INSERT VALUE !!!	
#define STEPS_PER_REVOLUTION 240
#define WHEEL_CIRCUMFERENCE (pi * WHEEL_DIAMETER)
#define IN_PER_STEP (WHEEL_CIRCUMFERENCE / STEPS_PER_REVOLUTION)

/*
 *LFM = Left Forward Motor
 *RFM = Right Forward Motor
 *LBM = Left Backward Motor
 *RBM = Right Backward Motor
*/


//Stepper Motors Setup///////////////////////////////////////////
Adafruit_MotorShield MS_top = Adafruit_MotorShield(0x60); //address attribution 
Adafruit_MotorShield MS_bot = Adafruit_MotorShield(0x61); //address attribution

//!Needs to be tuned for the exact motor order
Adafruit_StepperMotor *LFM = MS_bot.getStepper(200, 1);
Adafruit_StepperMotor *RFM = MS_bot.getStepper(200, 2);
Adafruit_StepperMotor *LBM = MS_top.getStepper(200, 1);
Adafruit_StepperMotor *RBM = MS_top.getStepper(200, 2);
/////////////////////////////////////////////////////////////////


//Motion Functions/////////////////////////////////////
void goBackward(double dist) { //d in inches
  int steps = dist / IN_PER_STEP;

  Serial.println("Moving backward");
  for (uint16_t i = 0; i <= steps ; ++i) {
    LFM->onestep(BACKWARD,DOUBLE);
    RFM->onestep(BACKWARD,DOUBLE);
    LBM->onestep(BACKWARD,DOUBLE);
    RBM->onestep(BACKWARD,DOUBLE);
  }
  //delay(20);
  Serial.println("Moved backward");
}

void goForward(double dist) { //d in inches
  int steps = dist / IN_PER_STEP;

  Serial.println("Moving forward");
  for (uint16_t i = 0; i <= steps ; ++i) {
    LFM->onestep(FORWARD,DOUBLE);
    RFM->onestep(FORWARD,DOUBLE);
    LBM->onestep(FORWARD,DOUBLE);
    RBM->onestep(FORWARD,DOUBLE);
  }
  //delay(20);
  Serial.println("Moved forward");
}

void goLeft(double dist) { //d in inches
  int steps = dist / IN_PER_STEP;

  Serial.println("Moving left");
  for (uint16_t i = 0; i <= steps ; ++i) {
    LFM->onestep(BACKWARD,DOUBLE);
    RFM->onestep(FORWARD,DOUBLE);
    LBM->onestep(FORWARD,DOUBLE);
    RBM->onestep(BACKWARD,DOUBLE);
  }
  //delay(20);
  Serial.println("Moved left");
}

void goRight(double dist) { //d in inches
  int steps = dist / IN_PER_STEP;
  Serial.println("Moving left");
  for (uint16_t i = 0; i <= steps ; ++i) {
    LFM->onestep(FORWARD,DOUBLE);
    RFM->onestep(BACKWARD,DOUBLE);
    LBM->onestep(BACKWARD,DOUBLE);
    RBM->onestep(FORWARD,DOUBLE);
  }
  //delay(20);
  Serial.println("Moved left");
}

void goUpLeft(double dist) { //d in inches
  int steps = dist / IN_PER_STEP;
  Serial.println("Moving up left");
  for (uint16_t i = 0; i <= steps ; ++i) {
    LFM->release();
    RFM->onestep(FORWARD,DOUBLE);
    LBM->onestep(FORWARD,DOUBLE);
    RBM->release();
  }
  //delay(20);
  Serial.println("Moved up left");
}

void goDownLeft(double dist) { //d in inches
  int steps = dist / IN_PER_STEP;
  Serial.println("Moving down left");
  for (uint16_t i = 0; i <= steps ; ++i) {
    LFM->onestep(BACKWARD,DOUBLE)
    RFM->release();
    LBM->release();
    RBM->onestep(BACKWARD,DOUBLE)
  }
  //delay(20);
  Serial.println("Moved down left");
}

void goUpRight(double dist) { //d in inches
  int steps = dist / IN_PER_STEP;
  Serial.println("Moving up right");
  for (uint16_t i = 0; i <= steps ; ++i) {
    LFM->onestep(FORWARD,DOUBLE)
    RFM->release();
    LBM->release();
    RBM->onestep(FORWARD,DOUBLE)
  }
  //delay(20);
  Serial.println("Moved up right");
}

void goDownRight(double dist) { //d in inches
  int steps = dist / IN_PER_STEP;
  Serial.println("Moving down right");
  for (uint16_t i = 0; i <= steps ; ++i) {
    LFM->release();
    RFM->onestep(BACKWARD,DOUBLE);
    LBM->onestep(BACKWARD,DOUBLE);
    RBM->release();
  }
  //delay(20);
  Serial.println("Moved down right");
}

void rotate(double angle) {   //angle in degrees

 int dist = (double)angle/ROTATE_ANGLE;
 //dist += ANGLE_CORRECTION_FACTOR;                           //correction factor
 dist *= 1.3;
 if(dist >= 0) {
  Serial.println("Rotating Clockwise");
  for (uint16_t i = 0; i <= dist ; ++i) {
    LFM->onestep(FORWARD,DOUBLE);
    RFM->onestep(BACKWARD,DOUBLE);
    LBM->onestep(FORWARD,DOUBLE);
    RBM->onestep(BACKWARD,DOUBLE);
  }
  //delay(20);
  Serial.println("Rotated Clockwise");
 }
 else { //dist < 0
  dist = -dist;
  Serial.println("Rotating Counterclockwise");
  for (uint16_t i = 0; i <= dist ; ++i) {
    LFM->onestep(BACKWARD,DOUBLE);
    RFM->onestep(FORWARD,DOUBLE);
    LBM->onestep(BACKWARD,DOUBLE);
    RBM->onestep(FORWARD,DOUBLE);
  }
  //delay(20);
  Serial.println("Rotated Counterclockwise");
 }
}

void goTo(double dist, double angle){				// distance is in inches
	double straight_angle = angle - 45;
	double rad_angle = pi / 180 * straight_angle;
	double x_dist = dist * cos(rad_angle);
	double y_dist = dist * sin(rad_angle);
	double x_steps = x_dist / IN_PER_STEP;
	double y_steps = y_dist / IN_PER_STEP;

	if (abs(x_steps) > abs(y_steps)){
		double speed_percent = abs(y_steps / x_steps);
		double slow_speed = speed_percent * MAX_SPEED;

		LFM->setSpeed(slow_speed);
		RFM->setSpeed(MAX_SPEED);
		LBM->setSpeed(MAX_SPEED);
		RBM->setSpeed(slow_speed);
	}
	else if (abs(x_steps) < abs(y_steps)){
		double speed_percent = abs(x_steps / y_steps);
		double slow_speed = speed_percent * MAX_SPEED;

		LFM->setSpeed(MAX_SPEED);
		RFM->setSpeed(slow_speed);
		LBM->setSpeed(slow_speed);
		RBM->setSpeed(MAX_SPEED);
	}
	else{
		LFM->setSpeed(MAX_SPEED);
		RFM->setSpeed(MAX_SPEED);
		LBM->setSpeed(MAX_SPEED);
		RBM->setSpeed(MAX_SPEED);
	}
		// might have to change some to negative based on the orientation of the motors
		LFM->step(x_steps);
		RFM->step(y_steps);
		LBM->step(y_steps);
		RBM->step(x_steps);
  
}


////////////////////////////////////////////////////

//Stage Function///////////////////////////////////
bool Stage1(bool A = 0) { //Executes Stage 1 depending on value of A from IR receiver
  if(A) {
    goRight((24 - HALF_ROBOT_LENGTH)- SAFETY_SENSOR_DISTANCE);
    //check Ultrasonic sensors 
    //check back and right
    //adjust approach
    goRight(SAFETY_SENSOR_DISTANCE); //Button Pressed
    goLeft(24 - HALF_ROBOT_LENGTH);
    //check sensor left, right, back
  }
  else {
    goLeft((24 - HALF_ROBOT_LENGTH)- SAFETY_SENSOR_DISTANCE);
    //check Ultrasonic sensors 
    //check back and left
    //adjust approach
    goLeft(SAFETY_SENSOR_DISTANCE); //Button Pressed
    goRight(24 - HALF_ROBOT_LENGTH);
    //check sensor left, right, back
  }
  //add a timer to make sure the robot is not stuck
  return true; //
}

bool failsafe(void) {
  //check if centered 
  //goForward(
}
//////////////////////////////////////////////////
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  MS_bot.begin();
  MS_top.begin();
  
  LFM->setSpeed(MAX_SPEED);
  RFM->setSpeed(MAX_SPEED);
  LBM->setSpeed(MAX_SPEED);
  RBM->setSpeed(MAX_SPEED);
  //check sensor for alignment
}

void loop() {
  // put your main code here, to run repeatedly:
  
}
