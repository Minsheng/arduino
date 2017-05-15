int potVal = 0;
int potPin = 0;

int ledcolor = 0;
int a = 200; //this sets how long the stays one color for
int red = 11; //this sets the red led pin
int green = 9; //this sets the green led pin
int blue = 10; //this sets the blue led pin
int changeRate = 0; // for change the color of RGB LED

void setup() {
  pinMode(red, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(blue, OUTPUT);
}

int redVal;
int blueVal;
int greenVal;
void loop() {
  potVal = analogRead(potPin);
  Serial.println(potVal);
  changeRate = map(potVal, 0, 1023, 0, 255);

  int redVal = changeRate;
  int blueVal = 0;
  int greenVal = 0;
  for( int i = 0 ; i < 255 ; i += 1 ){
    greenVal += 1;
    redVal -= 1;
    analogWrite( green, 255 - greenVal );
    analogWrite( red, 255 - redVal );

    delay( a );
  }
 
  redVal = 0;
  blueVal = 0;
  greenVal = changeRate;
  for( int i = 0 ; i < 255 ; i += 1 ){
    blueVal += 1;
    greenVal -= 1;
    analogWrite( blue, 255 - blueVal );
    analogWrite( green, 255 - greenVal );

    delay( a );
  }
 
  redVal = 0;
  blueVal = changeRate;
  greenVal = 0;
  for( int i = 0 ; i < 255 ; i += 1 ){
    redVal += 1;
    blueVal -= 1;
    analogWrite( red, 255 - redVal );
    analogWrite( blue, 255 - blueVal );

    delay( a );
  }
}
