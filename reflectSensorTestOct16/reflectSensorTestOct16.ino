void setup() {
  Serial.begin(115200);
}
 
// the loop routine runs over and over again forever:
void loop() {
  int s1 = analogRead(A0);
  int s2 = analogRead(A1);
  int s3 = analogRead(A2);
  int s4 = analogRead(A3);
  int s5 = analogRead(A4);

  int mean = (s1+s2+s3+s4+s5)/5;

//    Serial.print("Sensor 1");
  Serial.print(s1); Serial.print("  ");
//  Serial.print("Sensor 2");
  Serial.print(s2); Serial.print("  ");
//  Serial.print("Sensor 3");
  Serial.print(s3); Serial.print("  ");
//  Serial.print("Sensor 4");
  Serial.print(s4); Serial.print("  ");
//  Serial.print("Sensor 5");
  Serial.print(s5); Serial.print("  ");
  Serial.print("       ");
  Serial.print(s1-mean); Serial.print("  ");
  Serial.print(s2-mean); Serial.print("  ");
  Serial.print(s3-mean); Serial.print("  ");
  Serial.print(s4-mean); Serial.print("  ");
  Serial.print(s5-mean); Serial.print("  ");
 
  Serial.println();

  
  delay(1);        // delay in between reads for stability
}
