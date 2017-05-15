

int ledPin = 3; 
int blinkRate = 500;

long lastChange = 0;
boolean ledState = false;

// LED connected to digital pin 13

void setup()
{
  pinMode(ledPin, OUTPUT);      // sets the digital pin as output
}

void loop()
{
  if(millis()-lastChange>=blinkRate)
  {
    ledState = !ledState;
    digitalWrite(ledPin,ledState);
    lastChange=millis();  
  }
              
}
