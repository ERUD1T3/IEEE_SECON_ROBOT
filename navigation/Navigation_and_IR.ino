//////////////////////////////////////
//IEEE Robot 2018: Drive System
/////////////////////////////////////

#include <Adafruit_MotorShield.h> //Motor shield v2 
//#include "utility/Adafruit_MS_PWMServoDriver.h" //In case Servo is used 
#include <Wire.h>
#define MAX_SPEED 1000 //1200 RPM for Bipolar Stepper
#define ROTATE_ANGLE 0.6159936776 //
#define ANGLE_CORRECTION_FACTOR 5  //
#define HALF_ROBOT_LENGTH 6  //refers to the length of the robot from center to edge
#define SAFETY_SENSOR_DISTANCE 1 //maintain a safety distance of at least one inch before activating sensor for check
#define CORRECTION_FACTOR .4167
/*
 *Motor2 = Left Forward Motor
 *Motor4 = Right Forward Motor
 *Motor1 = Left Backward Motor
 *Motor3 = Right Backward Motor
*/

#define LOGIC_1 (2250) //logic one according to the rules
#define SIGNAL_TOLERANCE (100) //100 us tolerance in signal reception
#define LOGIC_0 (1120) //logic 0 accroding to the rules
#define START_PULSE (13500) //Start pulse according to the rules
#define N_PULSES_SAMPLE (25) //expected number of pulses for the msg

const byte A = 4;
const byte B = 7;
const byte C = 6;
const byte D = 5;

/*&segment pins*/
uint8_t BCDPin[4] = {A,B,C,D};  //pins for the ABCD decoder pins

const byte IR_pin = 2; //2 or 3 to use interrupts
long pulse_times[N_PULSES_SAMPLE] = {0}; //array of recorded times for the rising edge pulses
bool MsgIsValid = false; 
byte pulse_counter = 0; //counter of the number of pulses sample
byte check_idx = 0, check_seq[3]; //to make sure the signal received is correct


//Stepper Motors Setup///////////////////////////////////////////
Adafruit_MotorShield MS_top = Adafruit_MotorShield(0x60); //address attribution 
Adafruit_MotorShield MS_bot = Adafruit_MotorShield(0x61); //address attribution

//!Needs to be tuned for the exact motor order
Adafruit_StepperMotor *Motor2 = MS_bot.getStepper(200, 1); //inializing the pointer for the motors on the shield
Adafruit_StepperMotor *Motor4 = MS_bot.getStepper(200, 2); //inializing the pointer for the motors on the shield
Adafruit_StepperMotor *Motor1 = MS_top.getStepper(200, 1); //inializing the pointer for the motors on the shield
Adafruit_StepperMotor *Motor3 = MS_top.getStepper(200, 2);//inializing the pointer for the motors on the shield
/////////////////////////////////////////////////////////////////


//Motion Functions/////////////////////////////////////
void goBackward(int dist) { //d in millimeters
  dist *= 1.3;
  Serial.println("Moving backward");
  for (uint16_t i = 0; i <= dist ; ++i) {
    Motor2->onestep(BACKWARD,DOUBLE);
    Motor4->onestep(FORWARD,DOUBLE);
    Motor1->onestep(BACKWARD,DOUBLE);
    Motor3->onestep(FORWARD,DOUBLE);
  }
  //delay(20);
  Serial.println("Moved backward");
}

void goForward(int dist) { //d in millimeters
  dist *= 1.3;
  Serial.println("Moving forward");
  for (uint16_t i = 0; i <= dist ; ++i) {
    Motor2->onestep(FORWARD,DOUBLE);
    Motor4->onestep(BACKWARD,DOUBLE);
    Motor1->onestep(FORWARD,DOUBLE);
    Motor3->onestep(BACKWARD,DOUBLE);
  }
  //delay(20);
  Serial.println("Moved forward");
}

void goLeft(int dist) { //d in millimeters
  dist *= 1.3;
  Serial.println("Moving left");
  for (uint16_t i = 0; i <= dist ; ++i) {
    Motor2->onestep(FORWARD,DOUBLE);
    Motor4->onestep(FORWARD,DOUBLE);
    Motor1->onestep(BACKWARD,DOUBLE);
    Motor3->onestep(BACKWARD,DOUBLE);
  }
  //delay(20);
  Serial.println("Moved left");
}

void goRight(int dist) { //d in millimeters
  dist *= 1.3;
  Serial.println("Moving left");
  for (uint16_t i = 0; i <= dist ; ++i) {
    Motor2->onestep(BACKWARD,DOUBLE);
    Motor4->onestep(BACKWARD,DOUBLE);
    Motor1->onestep(FORWARD,DOUBLE);
    Motor3->onestep(FORWARD,DOUBLE);
  }
  //delay(20);
  Serial.println("Moved left");
}

void goUpLeft(int dist) { //d in millimeters
  dist *= 1.3;
  Serial.println("Moving up left");
  for (uint16_t i = 0; i <= dist ; ++i) {
    Motor1->release();
    Motor2->onestep(FORWARD,DOUBLE);
    Motor3->onestep(BACKWARD,DOUBLE);
    Motor4->release();
  }
  //delay(20);
  Serial.println("Moved up left");
}

void goDownLeft(int dist) { //d in millimeters
  dist *= 1.3;
  Serial.println("Moving down left");
  for (uint16_t i = 0; i <= dist ; ++i) {
    Motor1->onestep(BACKWARD,DOUBLE);
    Motor2->release();
    Motor3->release();
    Motor4->onestep(FORWARD,DOUBLE);
  }
  //delay(20);
  Serial.println("Moved down left");
}

void goUpRight(int dist) { //d in millimeters
  dist *= 1.3;
  Serial.println("Moving up right");
  for (uint16_t i = 0; i <= dist ; ++i) {
    Motor1->onestep(FORWARD,DOUBLE);
    Motor2->release();
    Motor3->release();
    Motor4->onestep(BACKWARD,DOUBLE);
  }
  //delay(20);
  Serial.println("Moved up right");
}

void goDownRight(int dist) { //d in millimeters
  dist *= 1.3;
  Serial.println("Moving down right");
  for (uint16_t i = 0; i <= dist ; ++i) {
     Motor1->release();
    Motor2->onestep(BACKWARD,DOUBLE);
    Motor3->onestep(FORWARD,DOUBLE);
    Motor4->release();
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
    Motor2->onestep(FORWARD,DOUBLE);
    Motor4->onestep(FORWARD,DOUBLE);
    Motor1->onestep(FORWARD,DOUBLE);
    Motor3->onestep(FORWARD,DOUBLE);
  }
  //delay(20);
  Serial.println("Rotated Clockwise");
 }
 else { //dist < 0
  dist = -dist;
  Serial.println("Rotating Counterclockwise");
  for (uint16_t i = 0; i <= dist ; ++i) {
    Motor2->onestep(BACKWARD,DOUBLE);
    Motor4->onestep(BACKWARD,DOUBLE);
    Motor1->onestep(BACKWARD,DOUBLE);
    Motor3->onestep(BACKWARD,DOUBLE);
  }
  //delay(20);
  Serial.println("Rotated Counterclockwise");
 }
}

////////////////////////////////////////////////////

//Stage Function///////////////////////////////////
bool Stage1(bool A = 0) { //Executes Stage 1 depending on value of A from IR receiver
  if(A) {
    goRight(((24 - HALF_ROBOT_LENGTH)- SAFETY_SENSOR_DISTANCE)*25.4);
    //check Ultrasonic sensors 
    //check back and right
    //adjust approach
    goRight(SAFETY_SENSOR_DISTANCE*25.4); //Button Pressed
    goLeft(((24 - HALF_ROBOT_LENGTH))*25.4);
    //check sensor left, right, back
  }
  else {
    goLeft(((24 - HALF_ROBOT_LENGTH)- SAFETY_SENSOR_DISTANCE)*25.4);
    //check Ultrasonic sensors 
    //check back and left
    //adjust approach
    goLeft(SAFETY_SENSOR_DISTANCE*25.4); //Button Pressed
    goRight(((24 - HALF_ROBOT_LENGTH))*25.4);
    //check sensor left, right, back
  }
  //add a timer to make sure the robot is not stuck
  return true; //
}

bool failsafe(void) {
  //check if centered 

}
//////////////////////////////////////////////////
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  MS_bot.begin();
  MS_top.begin();
  
  Motor2->setSpeed(MAX_SPEED);
  Motor4->setSpeed(MAX_SPEED);
  Motor1->setSpeed(MAX_SPEED);
  Motor4->setSpeed(MAX_SPEED);
  //check sensor for alignment

    for(uint8_t i = 0; i < 4; ++i)
    pinMode(BCDPin[i], OUTPUT);
    
  pinMode(IR_pin, INPUT); //IR_pin set as input
  attachInterrupt(digitalPinToInterrupt(IR_pin), Pulses, FALLING); //using external interrupt to detect falling edges             
  Serial.begin(2000000); //high baud rate to read appropriately the signal
}

void loop() {
  // put your main code here, to run repeatedly:
  byte to_disp = 0;
  if(pulse_counter == N_PULSES_SAMPLE) { //noise free pulse counter indicator
/*
    Serial.print("///READINGS////////////////////////////////////////////////////////////////////////////////////////////////////////////\n"); 
    
    for(uint8_t i = 0; i < N_PULSES_SAMPLE - 1; ++i) {
      Serial.print(" "); Serial.print(pulse_times[i+1] - pulse_times[i]); Serial.print("|"); //the read time intervales
    }
    
    Serial.print("\n----------------------------------------------------\n");
    
    for(uint8_t i = 0; i < N_PULSES_SAMPLE - 1; ++i) {
      Serial.print(" "); Serial.print(inRange((pulse_times[i+1] - pulse_times[i]), LOGIC_1, SIGNAL_TOLERANCE)); Serial.print("|");  //the corresponding binary sequence
    }
    Serial.println();
*/

   
    byte tmp = Sequence(pulse_times, N_PULSES_SAMPLE);
    check_seq[check_idx++] = tmp;
    //Serial.println(tmp);
    
    if(check_idx >= 3){
      if(allEqual(check_seq, 3)) {
        Serial.print("\nSequence: ");
        Serial.print(check_seq[2]);
        uint8_t msg = check_seq[2] & 0b00000111;
        Serial.print("\tMessage: ");
        Serial.print(msg);
        BCD_display(msg+1); to_disp = msg + 1;
        //BCD_display(5);
        Serial.println();
      }
      check_idx = 0;
      pulse_counter = 0; //reset the counter
      MsgIsValid = true;
    }
    //send message to master arduino then
    //if(MsgIsValid) detachInterrupt(IR_pin);
   
  }
  /*
  for(uint8_t i = 0; i <= 8; ++i) {
   BCD_display(i);
   delay(2000);
  }
  */
  if(MsgIsValid && to_disp > 0) {
    //delay(5000);
    if(to_disp == 1 || to_disp == 3) {
      goLeft(426*CORRECTION_FACTOR);
      goRight(426*CORRECTION_FACTOR);
    
      //goForward(1015*CORRECTION_FACTOR);
      //goBackward(1015*CORRECTION_FACTOR);
      delay(20000);
    
      goLeft(426*CORRECTION_FACTOR);
      goRight(426*CORRECTION_FACTOR);
    }

    if(to_disp == 2 || to_disp == 4) {
      goLeft(426*CORRECTION_FACTOR);
      goRight(426*CORRECTION_FACTOR);
    
      //goForward(1015*CORRECTION_FACTOR);
      //goBackward(1015*CORRECTION_FACTOR);
      delay(20000);
    
      goRight(426*CORRECTION_FACTOR);
      goLeft(426*CORRECTION_FACTOR);
    }

    if(to_disp == 5 || to_disp == 7) {
      goRight(426*CORRECTION_FACTOR);
      goLeft(426*CORRECTION_FACTOR);
    
      //goForward(1015*CORRECTION_FACTOR);
      //goBackward(1015*CORRECTION_FACTOR);
      delay(20000);
    
      goLeft(426*CORRECTION_FACTOR);
      goRight(426*CORRECTION_FACTOR);
    }

    if(to_disp == 6 || to_disp == 8) {
      goRight(426*CORRECTION_FACTOR);
      goLeft(426*CORRECTION_FACTOR);
    
      //goForward(1015*CORRECTION_FACTOR);
      //goBackward(1015*CORRECTION_FACTOR);
      delay(20000);
    
      goRight(426*CORRECTION_FACTOR);
      goLeft(426*CORRECTION_FACTOR);
    }
    MsgIsValid = false;
  }
  
  //goForward(50*CORRECTION_FACTOR);
  //goBackward(50*CORRECTION_FACTOR);
  //goLeft(50);
 // delay(2500);
  //goRight(50);
  //goUpLeft(50);
  //goDownLeft(50);
  //goUpRight(50);
  //goDownRight(50);
  //rotate(36);
  //delay(3600);
  //rotate(-36);
}

void Pulses(void) {
  pulse_times[pulse_counter++] = micros();
}

bool inRange(double value, double target, double tolerance) { //to determine if two values are equivalent within a range
  return (abs(value - target) <= tolerance); //to substitute equal
}

byte Sequence(long time_array[], byte array_size) { //print the bit sequence received 
  
  byte seq = 0x00; //empty sequence
  
  for(uint8_t i = 0; i < array_size -1; ++i) {
    if(inRange((pulse_times[i+1] - pulse_times[i]), START_PULSE, SIGNAL_TOLERANCE)) {
      Serial.println();
      for(uint16_t time_idx = i + 1, j = 7; time_idx < 9 + i; ++time_idx, --j ) {

        Serial.print("|"); 
        Serial.print(inRange((pulse_times[time_idx + 1] - pulse_times[time_idx]), LOGIC_1, SIGNAL_TOLERANCE));
        
        seq |= ((inRange((pulse_times[time_idx + 1] - pulse_times[time_idx]), LOGIC_1, SIGNAL_TOLERANCE)) << j); //record sequence
      }
      Serial.println();
      return seq;
    }
  } 
}

bool allEqual(auto Array, uint8_t Size) {
  for(uint8_t i = 0; i < Size; ++i) 
    if(Array[0] != Array[i]) 
      return false;
      
  return true;
}

void BCD_display(byte msg) {
  switch(msg) {
    case 0: {
      for(uint8_t i = 0; i < 4; ++i)
        digitalWrite(BCDPin[i], LOW);
    } break;
    case 1: {
      digitalWrite(BCDPin[0], HIGH);
      for(uint8_t i = 1; i < 4; ++i)
        digitalWrite(BCDPin[i], LOW);
    } break;
    case 2: {
      digitalWrite(BCDPin[1], HIGH);
      for(uint8_t i = 2; i < 4; ++i)
        digitalWrite(BCDPin[i], LOW);
      digitalWrite(BCDPin[0],LOW);
    } break;
    case 3:  {
      digitalWrite(BCDPin[0], HIGH);
      digitalWrite(BCDPin[1], HIGH);
      digitalWrite(BCDPin[2], LOW);
      digitalWrite(BCDPin[3], LOW);
    } break;
    case 4:  {
      digitalWrite(BCDPin[0], LOW);
      digitalWrite(BCDPin[1], LOW);
      digitalWrite(BCDPin[2], HIGH);
      digitalWrite(BCDPin[3], LOW);
    } break;
    case 5:  {
      digitalWrite(BCDPin[0], HIGH);
      digitalWrite(BCDPin[1], LOW);
      digitalWrite(BCDPin[2], HIGH);
      digitalWrite(BCDPin[3], LOW);
    } break;
    case 6:  {
       digitalWrite(BCDPin[0], LOW);
      digitalWrite(BCDPin[1], HIGH);
      digitalWrite(BCDPin[2], HIGH);
      digitalWrite(BCDPin[3], LOW);
    } break;
    case 7:  {
      digitalWrite(BCDPin[0], HIGH);
      digitalWrite(BCDPin[1], HIGH);
      digitalWrite(BCDPin[2], HIGH);
      digitalWrite(BCDPin[3], LOW);
    } break;
    case 8:  {
      digitalWrite(BCDPin[0], HIGH);
      digitalWrite(BCDPin[1], HIGH);
      digitalWrite(BCDPin[2], HIGH);
      digitalWrite(BCDPin[3], HIGH);
    } break;
  }
}
