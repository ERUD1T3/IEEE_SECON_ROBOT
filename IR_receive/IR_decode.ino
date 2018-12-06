//////////////////////////////////////////
//IEEE Robot 2018: IR receiver 
//////////////////////////////////////////
#define LOGIC_1 (2250) //logic one according to the rules
#define SIGNAL_TOLERANCE (100) //100 us tolerance in signal reception
#define LOGIC_0 (1120) //logic 0 accroding to the rules
#define START_PULSE (13500) //Start pulse according to the rules
#define N_PULSES_SAMPLE (25) //expected number of pulses for the msg
#define A 4
#define B 7
#define C 6
#define D 5

/*7segment pins*/
uint8_t BCDPin[4] = {A,B,C,D}; 

const byte IR_pin = 2; //2 or 3 to use interrupts
long pulse_times[N_PULSES_SAMPLE] = {0}; //array of recorded times for the rising edge pulses
bool MsgIsValid = false; 
byte pulse_counter = 0; //counter of the number of pulses sample
byte check_idx = 0, check_seq[3]; //to make sure the signal received is correct

void setup() {
  //put code here f
  for(uint8_t i = 0; i < 4; ++i) pinMode(BCDPin[i], OUTPUT); //setting the 4 decoder pins as output
    
  pinMode(IR_pin, INPUT); //IR_pin set as input
  attachInterrupt(digitalPinToInterrupt(IR_pin), Pulses, FALLING); //using external interrupt to detect falling edges             
  Serial.begin(2000000); //high baud rate to read appropriately the signal
}

void loop() {
  //put code here
  
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
   
    byte tmp = Sequence(pulse_times, N_PULSES_SAMPLE); //sample the sequence and store the temporary result
    check_seq[check_idx++] = tmp; //populate check_seq for sampling and error checking
    //Serial.println(tmp);
    
    if(check_idx >= 3){ //3 is enough to confirm the validity of sample
      if(allEqual(check_seq, 3)) { //if the 3 samples are identical the message is valid
        Serial.print("\nSequence: ");
        Serial.print(check_seq[2]);
        uint8_t msg = check_seq[2] & 0b00000111; //update the message to send variable
        Serial.print("\tMessage: ");
        Serial.print(msg);
        BCD_display(msg); //send the message
        Serial.println();
      }
      check_idx = 0; //reset for next sampling
      MsgIsValid = true; //previous message is valid
    }
    //send message to master arduino then
    pulse_counter = 0; //reset the counter
    //if(MsgIsValid) detachInterrupt(IR_pin); //stop sampling if message is satisfactory
   
  }
  /*
  for(uint8_t i = 0; i <= 8; ++i) {
   BCD_display(i);
   delay(2000);
  }
  */
}

void Pulses(void) { //populate array pulse time with times of pings
  pulse_times[pulse_counter++] = micros(); //collect times of infrared pins
}

bool inRange(double value, double target, double tolerance) { //to determine if two values are equivalent within a range
  return (abs(value - target) <= tolerance); //to substitute equal
}

byte Sequence(long time_array[], byte array_size) { //print the bit sequence received 
  
  byte seq = 0x00; //empty sequence
  
  for(uint8_t i = 0; i < array_size -1; ++i) {
    if(inRange((pulse_times[i+1] - pulse_times[i]), START_PULSE, SIGNAL_TOLERANCE)) { //if a start signal is detected
      Serial.println();
      for(uint16_t time_idx = i + 1, j = 7; time_idx < 9 + i; ++time_idx, --j ) {

        Serial.print("|"); 
        Serial.print(inRange((pulse_times[time_idx + 1] - pulse_times[time_idx]), LOGIC_1, SIGNAL_TOLERANCE));
        
        seq |= ((inRange((pulse_times[time_idx + 1] - pulse_times[time_idx]), LOGIC_1, SIGNAL_TOLERANCE)) << j); //record sequence
      }
      Serial.println();
      return seq; //return sequence
    }
  } 
}

bool allEqual(auto Array, uint8_t Size) { //return true if the sample in array are all the same 
  for(uint8_t i = 0; i < Size; ++i) 
    if(Array[0] != Array[i]) 
      return false;
      
  return true;
}

void BCD_display(byte msg) { //display number from 0 to 9 give a 4bits integer
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

