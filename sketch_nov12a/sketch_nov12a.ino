int valToSend = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    valToSend = Serial.read(); 
  }

  if (valToSend == 'H') {
    Serial.println("Daa receieved !");
  }
}
