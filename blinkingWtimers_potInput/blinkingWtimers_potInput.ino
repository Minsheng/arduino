

int ledPin = 3; 
int blinkRate = 500;

long lastChange = 0;
boolean ledState = false;

int potVal;

int potPin = 0;

// LED connected to digital pin 13

void setup()
{
  pinMode(ledPin, OUTPUT);      // sets the digital pin as output
  Serial.begin(9600);

}

void loop()
{

potVal = analogRead(potPin);

Serial.println(potVal);
  
  if(millis()-lastChange>=blinkRate)
  {
    ledState = !ledState;
    digitalWrite(ledPin,ledState);
    lastChange=millis();  
  }
              
}
