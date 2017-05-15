const int ledPin = 3;
const int buttonPin = 8;
const int potPin = 0;

int blinkRate = 500;
long lastChange = 0;
boolean ledState = false;
int potVal = 0;

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  potVal = analogRead(potPin);
  Serial.println(potVal);
  blinkRate = map(potVal, 0, 1023, 100, 800);
  
  if (millis() - lastChange >= blinkRate) {
    ledState = !ledState;
    digitalWrite(ledPin, ledState);
    lastChange = millis();
  }
}
