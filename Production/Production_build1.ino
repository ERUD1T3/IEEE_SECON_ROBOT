/////////////////////////////////////////////////////////
//IEEE Robot 2018: IR_decoding and time saving strategy
////////////////////////////////////////////////////////
/*<INSTRUCTIONS>
 * Set the Robot on top of the IR receiver and wait to 
 * detect the start sequence 7. The preceed to land the
 * robot frame on top of the transmitter. 
 * Wait few seconds for the signal to update to the actual
 * message.
 * Once message shown, touch the robot to end the round and
 * collect points for time management. 
 * </INSTRUCTIONS>
 */
 
#define LOGIC_1 (2250) //logic one according to the rules
#define SIGNAL_TOLERANCE (100) //100 us tolerance in signal reception
#define LOGIC_0 (1120) //logic 0 accroding to the rules
#define START_PULSE (13500) //Start pulse according to the rules
#define N_PULSES_SAMPLE (25) //expected number of pulses for the msg

/*&segment pins*/
uint8_t BCDPin[4] = {6,3,4,5}; 

const byte IR_pin = 2; //2 or 3 to use interrupts
long pulse_times[N_PULSES_SAMPLE] = {0}; //array of recorded times for the rising edge pulses
bool MsgIsValid = false; 
byte pulse_counter = 0; //counter of the number of pulses sample
byte check_idx = 0, check_seq[3]; //to make sure the signal received is correct

void setup() {
  //put code here f
  for(uint8_t i = 0; i < 4; ++i) pinMode(BCDPin[i], OUTPUT); //setting the BCD pins to OUTPUT
    
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
        BCD_display(msg);
        Serial.println();
      }
      check_idx = 0;
      MsgIsValid = true;
    }
    //send message to master arduino then
    pulse_counter = 0; //reset the counter
    if(MsgIsValid) detachInterrupt(IR_pin);
  }
}

void Pulses(void) {
  pulse_times[pulse_counter++] = micros(); //record pulse times in array
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
        
        seq |= ((inRange((pulse_times[time_idx + 1] - pulse_times[time_idx]), LOGIC_1, SIGNAL_TOLERANCE)) << j); //record sequence from MSB to LSB
      }
      Serial.println();
      return seq;
    }
  } 
}

bool allEqual(auto Array, uint8_t Size) { //ultility function for the reliability check of the message pulses
  for(uint8_t i = 0; i < Size; ++i) 
    if(Array[0] != Array[i]) 
      return false;    
  return true;
}

void BCD_display(byte msg) { //from a byte message, display the numerical equivalent on BCD display
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
