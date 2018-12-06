///////////////////////////////
//IEEE Robot 2018 
///////////////////////////////
//SINGLE SONAR 2/24/18
///////////////////////////////
#define DIST_FACTOR (0.034/2)

const int trigPin = 9; //PWM Pin for the pulse trigger
const int echoPin = 10; //PWM Pin for echo receive

class Sonar {
   public: 
    Sonar(int t_pin = 0, int e_pin = 0): trigPin(t_pin), echoPin(e_pin) {}

    void configurePin(void) { //has to be called in "void setup"
      pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
      pinMode(echoPin, INPUT); // Sets the echoPin as an Input
    }
    int distance_cm(void) {
      
      long ping(0); //duration of a single echo pulse
      int distance(0); //mesured distance from the echo pulse
      
      digitalWrite(trigPin, LOW); // Clears the trigPin
      delayMicroseconds(2);


      digitalWrite(trigPin, HIGH); // Sets the trigPin on HIGH state for 10 micro seconds
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW); //Reset the trigPin 

      ping = pulseIn(echoPin, HIGH); // Reads the echoPin, returns the sound wave travel time in microseconds

      distance = ping *DIST_FACTOR; // Calculating the distance
      // Prints the distance on the Serial Monitor
      
      return distance;
    }

  private:
    int trigPin;
    int echoPin;
};

Sonar sonar1(trigPin, echoPin);

void setup() {
  sonar1.configurePin();
  Serial.begin(9600); // Starts the serial communication
}

void loop() {
  
  Serial.print("Distance in centimeters: ");
  Serial.println(sonar1.distance_cm());
}
