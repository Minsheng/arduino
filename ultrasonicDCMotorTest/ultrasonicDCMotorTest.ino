//Since we are using servos and ultrasonic sensors in the robot we will include some libraries written to make their use easier
#include <Servo.h>
#include <NewPing.h>

//Below are the symbolic constants. Instead of having to type in a non-sensical pin number each time we want to do something we can write an easy to understand name which represents the pin, the compiler will then replace the names with the numbers

#define LeftMotor 3
#define ServoPin 6
#define USTrigger 8
#define USEcho 9
#define MaxDistance 100
#define LEDPin 13

//Here we have created two 'objects', one for the servo and one for the ultrasonic sensor
Servo servo;
NewPing sonar(USTrigger, USEcho, MaxDistance);

//Below we are creating unsigned integer variables which we will use later on in the code. They are unsigned as they will only have postive values
unsigned int duration;
unsigned int distance;
unsigned int FrontDistance;
unsigned int LeftDistance;
unsigned int RightDistance;
unsigned int Time;
unsigned int CollisionCounter;

void setup()                                            //This block happens once on startup
{
  Serial.begin(9600);                              //I have included the serial initialize but commented it out, if you want to debug and print information to the serial monitor just uncomment
  //Here we are setting the pin modes. As we will sending out signals from the pins we set them as outputs
  pinMode(LeftMotor, OUTPUT);
  pinMode(LEDPin, OUTPUT);
  servo.attach(ServoPin);                                    //The servo is attached to pin 4

  analogWrite(LeftMotor, 80); // Debug
}

void loop()                                           //This block repeats itself while the Arduino is turned on
{
  servo.write(90);                                    //Rotate the servo to face the front                  
  scan();                                             //Go to the scan function
  FrontDistance = distance;                           //Set the variable FrontDistance to the value of the distance returned from the scan function
  Serial.println("Front distance = ");
  Serial.print(distance);
  if(FrontDistance > 40 || FrontDistance == 0)        //If there is nothing infront of the robot within 40cm or the distance value is 0 (which for the newping libary means no ping was returned) then...
  {
   moveForward();                                     //Go to the moveForward function
  } 
  else                                                //Else (if there is something infront of the robot within 40cm) then...
  {
    CollisionCounter = CollisionCounter + 1;
    moveStop();                                       //Go to the moveStop function
    navigate();
  }
}

void moveForward()                                    //This function tells the robot to go forward 
{
  Serial.println("");
  Serial.println("Moving forward");
//  digitalWrite(LeftMotorBackward, LOW);
//  digitalWrite(LeftMotorForward, HIGH);
//  digitalWrite(RightMotorBackward, LOW);
//  digitalWrite(RightMotorForward, HIGH);
}

void moveBackward()                                  //This function tells the robot to move backward
{
  Serial.println("");
  Serial.println("Moving backward");
//  digitalWrite(LeftMotorForward, LOW);
//  digitalWrite(LeftMotorBackward, HIGH);
//  digitalWrite(RightMotorForward, LOW);
//  digitalWrite(RightMotorBackward, HIGH);
}

void moveLeft()                                      //This function tells the robot to turn left
{
  Serial.println("");
  Serial.println("Moving left");
//  digitalWrite(LeftMotorForward, LOW);
//  digitalWrite(LeftMotorBackward, HIGH);
//  digitalWrite(RightMotorBackward, LOW);
//  digitalWrite(RightMotorForward, HIGH);
  
}

void moveRight()                                    //This function tells the robot to turn right
{
  Serial.println("");
  Serial.println("Moving right");
//  digitalWrite(LeftMotorBackward, LOW);
//  digitalWrite(LeftMotorForward, HIGH);
//  digitalWrite(RightMotorForward, LOW);
//  digitalWrite(RightMotorBackward, HIGH);
}

void moveStop()                                     //This function tells the robot to stop moving
{
  Serial.println("");
  Serial.println("Stopping");
//  digitalWrite(LeftMotorBackward, LOW);
//  digitalWrite(LeftMotorForward, LOW);
//  digitalWrite(RightMotorForward, LOW);
//  digitalWrite(RightMotorBackward, LOW);
}
void scan()                                         //This function determines the distance things are away from the ultrasonic sensor
{
  Serial.println("");
  Serial.println("Scanning");
  Time = sonar.ping(); 
  distance = Time / US_ROUNDTRIP_CM;
  delay(500);
}
void navigate()
{
    Serial.println("There's an obstacle!");
    servo.write(167);                                 //Move the servo to the left (my little servos didn't like going to 180 so I played around with the value until it worked nicely)
    delay(1000);                                       //Wait half a second for the servo to get there
    scan();                                           //Go to the scan function
    LeftDistance = distance;                          //Set the variable LeftDistance to the distance on the left
    Serial.println("Left distance = ");
    Serial.print(distance);
    servo.write(0);                                   //Move the servo to the right
    delay(1000);                                       //Wait half a second for the servo to get there
    scan();                                           //Go to the scan function
    RightDistance = distance;                         //Set the variable RightDistance to the distance on the right
    Serial.println("Right distance = ");
    Serial.print(distance);
    if(abs(RightDistance - LeftDistance) < 5)
    {
      moveBackward();                                  //Go to the moveBackward function
      delay(200);                                      //Pause the program for 200 milliseconds to let the robot reverse
      moveRight();                                     //Go to the moveRight function
      delay(100);                                      //Pause the program for 200 milliseconds to let the robot turn right
    }
    else if(RightDistance < LeftDistance)                  //If the distance on the right is less than that on the left then...
    {
     moveLeft();                                      //Go to the moveLeft function
     delay(100);                                      //Pause the program for half a second to let the robot turn
    }
    else if(LeftDistance < RightDistance)             //Else if the distance on the left is less than that on the right then...
    {
     moveRight();                                     //Go to the moveRight function
     delay(100);                                      //Pause the program for half a second to let the robot turn
    }
}

