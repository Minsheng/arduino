#include <Wire.h>
#include <LIDARLite.h>

LIDARLite myLidarLite;

// in cm
int minDist = 200;
int maxDist = 300;

void setup() {
  Serial.begin(115200);
  myLidarLite.begin();
}

void loop() {
  int beamVal = myLidarLite.distance();
//  Serial.println(beamVal);

  int rowRx = 0;
  
  if ((beamVal >= minDist) && (beamVal <= maxDist)) {
    rowRx = map(beamVal, minDist, maxDist, 8, 0);
  } else {
    // all move
    rowRx = 0;
  }

  Serial.print("The distance of the object is ");
  Serial.println(rowRx);
}
