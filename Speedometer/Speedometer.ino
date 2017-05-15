
// Motor and Potentiometer
int potPin = A4;        int potValue = 0;
int motorPin = 3;       int motorValue = 0;

// Rotary Wheel-Spin Sensor
int sensorRotaryPin = A0;
int sensorVal = 0;           // 0 or 1 depending on whether the IR sensor detects the spokes or passes through them
int previousSensorVal = 0;   // previous 0/1 value to use during loop calculations
int count = 0;               // used for counting times digital signal shifts btw 0 and 1
float revolutions = 0;       // number of times the wheel has turned 360 degrees
float rpm = 0;               // rotations per minute
float speedVal = 0;          // distance/time val calculated using revolutions and constants
float minute = 60000;
int samplingInterval = 1000; // sample period of wheel turns over milliseconds to determine speed
long previousMillis = 0;


void setup() {
Serial.begin(250000);
pinMode(8, INPUT);
}

void loop() {

potValue = analogRead(potPin); if(potValue>1023) {potValue=1023;}                
motorValue = map(potValue, 0, 1023, 0, 255); analogWrite(motorPin, motorValue);  
sensorVal = digitalRead(sensorRotaryPin);        

spokesCounter(); 
GetSpeed(); 





// Value Printing
Serial.print(potValue);     Serial.print("   ");        
Serial.print(motorValue);   Serial.print("   ");  
Serial.print(sensorVal);    Serial.print("    ");
Serial.print(count);        Serial.print("    ");
Serial.print(revolutions);    Serial.print("     ");
Serial.println(rpm);


}



// Functions
int spokesCounter() {   // Counts the number of times that analogRead changes values. A count of 20 spokes is 360 degrees.
  if (sensorVal != previousSensorVal) {
    count++;                 
  }
  previousSensorVal = sensorVal;  // sets previous sensor val for next iteration through loop to compare
  }


float GetSpeed() {
  if(millis() - previousMillis >= samplingInterval) {   // uses timer to turn count() value into rotation values and speed data
    revolutions = count/20.;  // # of revolutions. 20 spokes. If 20 changes then the wheel has gone around 360 degree and revolutions = 1
    rpm = revolutions * (60000.0/samplingInterval); // revolutions per minute (60,000 ms)
    speedVal = (rpm * 20.0); // 20 cm is the wheel's circumference. speed = cm/minute
    previousMillis = millis(); // reset timer to resample
    count = 0; // reset counter for next sample
  }
}






