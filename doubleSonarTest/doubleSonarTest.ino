#include <NewPing.h>
#define INFO 0 // if INFO is on, print debug info

/* For distance sensor, INPUT */
#define UPPER_ECHO_PIN 10
//#define UPPER_TRIG_PIN 3
#define LOWER_ECHO_PIN 11
//#define LOWER_TRIG_PIN 5
#define MAX_DISTANCE 80
#define MAX_STOP_DISTANCE 20

/* Define distance sensor for obstacle detection */
NewPing upperSonar(UPPER_ECHO_PIN, UPPER_ECHO_PIN, MAX_DISTANCE); // for detecting wall
NewPing lowerSonar(LOWER_ECHO_PIN, LOWER_ECHO_PIN, MAX_DISTANCE); // for detecting ping pong ball
unsigned int upperReading; // distance raw value read from upper sonar
unsigned int lowerReading; // distance raw value read from lower sonar
unsigned int pingSpeed = 50; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
unsigned long pingTimer;     // Holds the next ping time.

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  pingTimer = millis(); // Start now.
  if (millis() >= pingTimer) {
    pingTimer += pingSpeed;
  
    // read from the upper sonar sensor
    // get a good sample, the larger the sample the longer it takes to read
    upperReading = upperSonar.ping_median(5);
  
    // convert to cm
    int upperDistanceCM = (int)upperReading / US_ROUNDTRIP_CM;
  
    if (upperDistanceCM > 0 && upperDistanceCM <= MAX_STOP_DISTANCE) {
      if (INFO == 1) {
        Serial.print("Upper Sensor Distance: ");
        Serial.print(upperDistanceCM);
        Serial.println();
      }
      Serial.println("Wall detected...");
    } else {
      Serial.print("Upper Sensor Distance: ");
      Serial.print(upperDistanceCM);
      Serial.println();
    }
    
    lowerReading = lowerSonar.ping_median(5);
    int lowerDistanceCM = (int)lowerReading / US_ROUNDTRIP_CM;

    if (lowerDistanceCM > 0 && lowerDistanceCM <= MAX_STOP_DISTANCE) {
      if (INFO == 1) {
        Serial.print("Lower Sensor Distance: ");
        Serial.print(lowerDistanceCM);
        Serial.println();
      }
      Serial.println("Grabbing the ball...");
    } else {
      Serial.print("Lower Sensor Distance: ");
      Serial.print(lowerDistanceCM);
      Serial.println();
    }
  }
}
