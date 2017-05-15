//motor A connected between A01 and A02 - Left Motor
//motor B connected between B01 and B02 - Right Motor
char val;
int STBY = 10; //standby

//Motor A
int PWMA = 3; //Speed control 
int AIN1 = 9; //Direction
int AIN2 = 8; //Direction

//Motor B
int PWMB = 5; //Speed control
int BIN1 = 11; //Direction
int BIN2 = 12; //Direction

void setup(){
  pinMode(STBY, OUTPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  Serial.begin(9600);
}

void loop(){
if (Serial.available())
   { // If data is available to read,
   val = Serial.read(); // read it and store it in val
   }
  
if (val == 'R') {
   move(1, 255, 0); 
   move(2, 255, 1);
   }

if (val == 'L') {
   move(1, 255, 1); 
   move(2, 255, 0);
   }

if (val == 'F') {
   move(1, 255, 0); 
   move(2, 255, 0);
   }
  
if (val == 'B') {
   move(1, 100, 1); 
   move(2, 100, 1);
  }
else {
  stop();
}
}


void move(int motor, int speed, int direction){
//Move specific motor at speed and direction
//motor: 0 for B 1 for A
//speed: 0 is off, and 255 is full speed
//direction: 0 clockwise, 1 counter-clockwise

  digitalWrite(STBY, HIGH); //disable standby

  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;

  if(direction == 1){
    inPin1 = HIGH;
    inPin2 = LOW;
  }

  if(motor == 1){
    digitalWrite(AIN1, inPin1);
    digitalWrite(AIN2, inPin2);
    analogWrite(PWMA, speed);
  }else{
    digitalWrite(BIN1, inPin1);
    digitalWrite(BIN2, inPin2);
    analogWrite(PWMB, speed);
  }
}

void stop(){
//enable standby  
  digitalWrite(STBY, LOW); 
}
