#include <NewPing.h>

#define ALARM_PIN 9
#define ECHO_PIN 11
#define TRIG_PIN 12
// For ATTiny
//#define ALARM_PIN 8
//#define ECHO_PIN 9
//#define TRIG_PIN 10
#define MAX_DISTANCE 80
#define ALARM_LONG 262 // frequency of alarm sound long
#define ALARM_MED 523 // frequency of alarm sound medium
#define ALARM_SHORT 1047 // frequency of alarm sound short

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);  
int noteDuration = 40; // set the duratoin for the alarm sound
unsigned int uS; // distance raw value read from sonar

void setup()
{
  Serial.begin(115200);
  pinMode(ALARM_PIN, OUTPUT);
}

void loop()
{
  int k, l = 13;
  Serial.println(k);
  Serial.println(l);
  // get a good sample
  uS = sonar.ping_median(10); 

  // convert to cm
  int cm = (int)uS / US_ROUNDTRIP_CM;

  if (cm == 0) {
    Serial.println("Out of range.");
    alarm(10); // if out of range, set distance so alarm sound is played anyway
  } else {
    // Debug print raw result.
    Serial.print("Ping: ");
    Serial.print(cm);
    Serial.println("cm");
    alarm(cm);
  }
}

void alarm(int distance) {
  // set a threshold for the delay time between each buzz
  int pauseBetweenNotes = 0;
  int alarmNote = 0;
  if (distance <= 10) {
    pauseBetweenNotes = 1;
    alarmNote = ALARM_SHORT;
  } else if (distance > 10 and distance <=30) {
    pauseBetweenNotes = 200;
    alarmNote = ALARM_MED;
  } else {
    pauseBetweenNotes = 1000;
    alarmNote = ALARM_LONG;
  }
  
  // start playing
  buzz(ALARM_PIN, alarmNote, noteDuration);
  
  delay(pauseBetweenNotes);

  // stop playing:
  buzz(ALARM_PIN, 0, noteDuration);
}

void buzz(int targetPin, long frequency, long length) {
  long delayValue = 1000000 / frequency / 2; // calculate the delay value between transitions
  //// 1 second's worth of microseconds, divided by the frequency, then split in half since
  //// there are two phases to each cycle
  long numCycles = frequency * length / 1000; // calculate the number of cycles for proper timing
  //// multiply frequency, which is really cycles per second, by the number of seconds to
  //// get the total number of cycles to produce
  for (long i = 0; i < numCycles; i++) { // for the calculated length of time...
    digitalWrite(targetPin, HIGH); // write the buzzer pin high to push out the diaphram
    delayMicroseconds(delayValue); // wait for the calculated delay value
    digitalWrite(targetPin, LOW); // write the buzzer pin low to pull back the diaphram
    delayMicroseconds(delayValue); // wait again or the calculated delay value
  }
}
