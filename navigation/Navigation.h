//////////////////////////////////////
//IEEE Robot 2018: Drive System 
/////////////////////////////////////
#include "Sonar.h"
#include <Adafruit_MotorShield.h> //Motor shield v2 
//#include "utility/Adafruit_MS_PWMServoDriver.h" //In case Servo is used 
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define MAX_SPEED 1200 //1200 RPM for Bipolar Stepper
#define ROTATE_ANGLE 0.6159936776 //
#define ANGLE_CORRECTION_FACTOR 5  //
#define HALF_ROBOT_LENGTH 6  //refers to the length of the robot from center to edge
#define SAFETY_SENSOR_DISTANCE 1 //maintain a safety distance of at least one inch before activating sensor for check
#define BNO055_SAMPLERATE_DELAY_MS (100)
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
  Sonar front_sonar(9,10); //sonar in front of the robot
  Sonar left_sonar(7,8); //sonar on the left of the robot
  Sonar right_sonar(5,6); //sonar on the right of the robot
  Sonar back_sonar(3,4); //sonar on the back of the robot
//////////////////////////////////////////////////////////////////
Adafruit_BNO055 gyro = Adafruit_BNO055(); //gyroscope for robot

//UTILITIES
int pythagoran(int x, int y) {
  return (sqrt(sq(x)+ sq(y)));
}

/////////////////////////////////////////
//LEVEL 1 FUNCTON 
void goBackward(int dist) { //d in millimeters
  dist *= 1.3;
  Serial.println("Moving backward");
  for (uint16_t i = 0; i <= dist ; ++i) {
    LFM->onestep(BACKWARD,DOUBLE);
    RFM->onestep(BACKWARD,DOUBLE);
    LBM->onestep(BACKWARD,DOUBLE);
    RBM->onestep(BACKWARD,DOUBLE);
  }
  //delay(20);
  Serial.println("Moved backward");
}

void goForward(int dist) { //d in millimeters
  dist *= 1.3;
  Serial.println("Moving forward");
  for (uint16_t i = 0; i <= dist ; ++i) {
    LFM->onestep(FORWARD,DOUBLE);
    RFM->onestep(FORWARD,DOUBLE);
    LBM->onestep(FORWARD,DOUBLE);
    RBM->onestep(FORWARD,DOUBLE);
  }
  //delay(20);
  Serial.println("Moved forward");
}

void goLeft(int dist) { //d in millimeters
  dist *= 1.3;
  Serial.println("Moving left");
  for (uint16_t i = 0; i <= dist ; ++i) {
    LFM->onestep(BACKWARD,DOUBLE);
    RFM->onestep(FORWARD,DOUBLE);
    LBM->onestep(FORWARD,DOUBLE);
    RBM->onestep(BACKWARD,DOUBLE);
  }
  //delay(20);
  Serial.println("Moved left");
}

void goRight(int dist) { //d in millimeters
  dist *= 1.3;
  Serial.println("Moving left");
  for (uint16_t i = 0; i <= dist ; ++i) {
    LFM->onestep(FORWARD,DOUBLE);
    RFM->onestep(BACKWARD,DOUBLE);
    LBM->onestep(BACKWARD,DOUBLE);
    RBM->onestep(FORWARD,DOUBLE);
  }
  //delay(20);
  Serial.println("Moved left");
}

void goUpLeft(int dist) { //d in millimeters
  dist *= 1.3;
  Serial.println("Moving up left");
  for (uint16_t i = 0; i <= dist ; ++i) {
    LFM->release();
    RFM->onestep(FORWARD,DOUBLE);
    LBM->onestep(FORWARD,DOUBLE);
    RBM->release();
  }
  //delay(20);
  Serial.println("Moved up left");
}

void goDownLeft(int dist) { //d in millimeters
  dist *= 1.3;
  Serial.println("Moving down left");
  for (uint16_t i = 0; i <= dist ; ++i) {
    LFM->onestep(BACKWARD,DOUBLE);
    RFM->release();
    LBM->release();
    RBM->onestep(BACKWARD,DOUBLE);
  }
  //delay(20);
  Serial.println("Moved down left");
}

void goUpRight(int dist) { //d in millimeters
  dist *= 1.3;
  Serial.println("Moving up right");
  for (uint16_t i = 0; i <= dist ; ++i) {
    LFM->onestep(FORWARD,DOUBLE);
    RFM->release();
    LBM->release();
    RBM->onestep(FORWARD,DOUBLE);
  }
  //delay(20);
  Serial.println("Moved up right");
}

void goDownRight(int dist) { //d in millimeters
  dist *= 1.3;
  Serial.println("Moving down right");
  for (uint16_t i = 0; i <= dist ; ++i) {
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

/////////////////////////////////////////////
//LEVEL 2 FUNCTION
bool move_XY(uint8_t x, uint8_t y, double phi = 0) {
  if(x == 0 && y == 0) {
    rotate(phi);
  } else if(y == 0) {
    (x > 0)? goRigth(x): goLeft(abs(x));
  } else if (x == 0) {
    (y > 0)? goForward(y): goBackward(abs(y));
  } else if ( x > 0 && y > 0) {
    bool combo_move = false;
    (abs(x) == abs(y))? goUpRight(pythagoran(x,y)): combo_move = true;
  } else if ( x < 0 && y > 0) {
    bool combo_move = false;
    (abs(x) == abs(y))? goUpLeft(pythagoran(x,y)): combo_move = true;
  } else if ( x < 0 && y < 0) {
    bool combo_move = false;
    (abs(x) == abs(y))? goDownLeft(pythagoran(x,y)): combo_move = true;
  } else if ( x > 0 && y < 0) {
    bool combo_move = false;
    (abs(x) == abs(y))? goDownRight(pythagoran(x,y)): combo_move = true;
  } 
  return true;
}

void sensors(uint8_t& front, uint8_t& left, uint8_t& right, uint8_t& back, double& angle_x) {
  imu::Vector<3> euler = gyro.getVector(Adafruit_BNO055::VECTOR_EULER); // set the measurement to be absolute angle
  front = front_sonar.distance_cm();
  left = left_sonar.distance_cm();
  rigt = right_sonar.distance_cm();
  back = back_sonar.distance_cm();
  angle_x = euler.x();
}

/////////////////////////////////////////////////////
//LEVEL 3 //under construction

////////////////////////////////////////////////////

void config_nav_level1(uint8_t speed) { //needs to be called in the setup 
  MS_bot.begin(); //init top shield
  MS_top.begin(); //init bottom shield
  
  front_sonar.configurePin(); //init front sonar
  left_sonar.configurePin(); //init left sonar
  right_sonar.configurePin(); //init right sonar
  back_sonar.configurePin(); //init back sonar

  gyro_sensor.begin(); //init gyroscope

//Setting speed of motors
  LFM->setSpeed(MAX_SPEED); 
  RFM->setSpeed(MAX_SPEED);
  LBM->setSpeed(MAX_SPEED);
  RFM->setSpeed(MAX_SPEED);

}

bool failsafe(void) {
  //check time to tell if current process needs to be killed 
}
