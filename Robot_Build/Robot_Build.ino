#include <NewPing.h>

#define SONAR_NUM     1 // No of sensors - change this if you want to add more sensors.
#define MAX_DISTANCE 500 // Maxi distance (in cm) to ping
#define PING_INTERVAL 250 // Ms ping-ping

unsigned long pingTimer[SONAR_NUM];
unsigned int cm[SONAR_NUM];
uint8_t currentSensor = 0;

NewPing sonar[SONAR_NUM] = {
  NewPing(7, 6, MAX_DISTANCE), // Trigger pin, echo pin, and max. Copy & paste this line (changing trig & echo pins) for each sensor you physically use.
};

void setup() {
  Serial.begin(9600);
  pingTimer[0] = millis() + 75;
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the start time for each sensor
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;

  //Setup Channel A & B
  pinMode(12, OUTPUT); //Motor Channel A pin
  pinMode(9, OUTPUT); //Brake Channel A pin
  pinMode(13, OUTPUT); //Motor Channel B pin
  pinMode(8, OUTPUT); //Brake Channel B pin  
}

void loop() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all sensors
    if (millis() >= pingTimer[i]) {         // Check each sensor for time to ping
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set time for next ping.
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); 
      sonar[currentSensor].timer_stop();       
      currentSensor = i;
      cm[currentSensor] = MAX_DISTANCE;               
      sonar[currentSensor].ping_timer(echoCheck); 
    }
  }
//    for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.

       if (cm[0] < 20)  // something seen by right US
       {
          turn_right();
       } 
       
//       else if(cm[1]< 20) // something seen by left US. If adding a 2nd sensor, this code can be used to call a different reaction.
//       {
//          turn_left(); // turn left  
//       } 

       else; // if there's nothing in front of us
       {
          forward(); // drive forward        
       }
  
}

void echoCheck() {
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

void oneSensorCycle() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    Serial.print(i);
    Serial.print("=");
    Serial.print(cm[i]);
    Serial.print("cm ");
  }
  Serial.println();
}
void forward()
{
  //forward @ full speed
  digitalWrite(12, LOW);  //Setting direction of Channel A
  digitalWrite(9, LOW);   //Disengage the Brake for Channel A
  analogWrite(3, 255);    //Spins the motor on Channel A at full speed
  digitalWrite(13, HIGH); //Setting direction of Channel B 
  digitalWrite(8, LOW);   //Disengage the Brake for Channel B
  analogWrite(11, 255);   //Spins the motor on Channel B at full speed
}
//void turn_left()
//{
  //turn left
//  digitalWrite(12, LOW);  //Setting direction of Channel A
//  digitalWrite(9, LOW);   //Disengage the Brake for Channel A
//  analogWrite(3, 255);    //Spins the motor on Channel A at low speed
//  digitalWrite(13, LOW);  //Setting direction of Channel B
//  digitalWrite(8, LOW);   //Disengage the Brake for Channel B
//  analogWrite(11, 255);   //Spins the motor on Channel B at full speed
//  delay(500);
//}
void turn_right()
{ 
  //turn right
  digitalWrite(12, HIGH); //Setting direction of Channel A 
  digitalWrite(9, LOW);   //Disengage the Brake for Channel A
  analogWrite(3, 255);    //Spins the motor on Channel A at full speed
  digitalWrite(13, HIGH); //Setting  direction of Channel B
  digitalWrite(8, LOW);   //Disengage the Brake for Channel B
  analogWrite(11, 255);   //Spins the motor on Channel B at low speed
  delay(500);
}
