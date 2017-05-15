#include <QTRSensors.h>

#define INFO 1

/* For QTR sensors, INPUT */
#define NUM_SENSORS 6 // number of sensors used
#define NUM_SAMPLES_PER_SENSOR 4 // average 4 analog samples per sensor reading
#define MAX_SPEED 100 // maximum when driving

/* For PID control */
#define Kp 0.05 // proportional constant
#define Ki 0.0001 // integral constant
#define Kd 1.5 // derivative constant

int lastError = 0;

int inByte = 0;         // incoming serial byte

/* sensors 0 through 5 are connected to analog inputs 0 through 5, respectively */
QTRSensorsAnalog qtra((unsigned char[]) {0, 1, 2, 3, 4, 5}, 
  NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, QTR_NO_EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

void setup() {
  Serial.begin(9600);
  init_calibrate(20, 100);
  establish_contact();
}

void loop() {
  line_follow_drive();
}

void establish_contact() {
  while (Serial.available() <= 0) {
    Serial.print('A');   // send a capital A
    delay(100);
  }
}

void send_data(int leftVal, int rightVal) {
  if (Serial.available() > 0) {
    // get incoming byte:
    inByte = Serial.read();
    // read first analog input, divide by 4 to make the range 0-255:
    
    // delay 10ms to let the ADC recover:
    delay(10);
    // send sensor values:
//    Serial.println("Sending left motor value...");
    Serial.write(leftVal); delay(10);
//    Serial.println("Sending right motor value...");
    Serial.write(rightVal); delay(10);
//    Serial.print(firstSensor); Serial.print(secondSensor); Serial.println(thirdSensor);
  }
}

void line_follow_drive() {
  unsigned int position = 0;
  long integral = 0;
  int powerDifference = 0;
  int leftMotorSpeed = 0;
  int rightMotorSpeed = 0;
  
  // get calibrated readings along with the line position, 
  // refer to the QTR Sensors Arduino Library for more details on line position.
  position = qtra.readLine(sensorValues);
  
//  Serial.println(position);
  
  // The "error" term should be 0 when we are on the line.
  // i.e., if there are 6 sensors and the readings are from (0~5000),
  // the error will be -2500 ~ 2500
  int error = (int) position - 3000;

  // Compute the derivative (change), which determines the rate of change of
  // the error value
  int derivative = error - lastError;

  // Compute the integral (sum) of the previous position values,
  // which records the history of the robot's motion
  integral += error;

  // Remember the last position of the robot
  lastError = error;

//  if (INFO == 1) {
//    Serial.print("Current Error: ");
//    Serial.print(error);
//    Serial.print(" | ");
//    Serial.print("Integral: ");
//    Serial.print(integral);
//    Serial.print(" | ");
//    Serial.print("Deriva: ");
//    Serial.print(derivative);
//    Serial.println();
//  }

  // Compute the difference between the two motor power settings,
  // m1 - m2.  If this is a positive number the robot will turn
  // to the right.  If it is a negative number, the robot will
  // turn to the left, and the magnitude of the number determines
  // the sharpness of the turn.
  powerDifference = error*Kp + integral*Ki + derivative*Kd;

  // Aggressive turning alogrithm, needs to be tested out
//  if (derivative <= -1300 || derivative >= 1300) {
//    Serial.println("Big Gap!");
//    powerDifference = error*aggrKp + integral*aggrKi + derivative*aggrKd;
//  } else {
//    powerDifference = error*Kp + integral*Ki + derivative*Kd;
//  }
  
//  if (INFO == 1) {
//    Serial.println(powerDifference);
//  }
  
  // Compute the actual motor settings.  We never set either motor
  // to a negative value.
  if(powerDifference > MAX_SPEED)
    powerDifference = MAX_SPEED;
  
  if(powerDifference < -MAX_SPEED)
    powerDifference = -MAX_SPEED;

  if (powerDifference < 0) {
    leftMotorSpeed = MAX_SPEED+powerDifference;
    rightMotorSpeed = MAX_SPEED;
  } else {
    leftMotorSpeed = MAX_SPEED;
    rightMotorSpeed = MAX_SPEED-powerDifference;
  }
  
//  set_motors(leftMotorSpeed, rightMotorSpeed);
  send_data(leftMotorSpeed, rightMotorSpeed);
//  if (INFO == 1) {
//    Serial.print(leftMotorSpeed);
//    Serial.print('\t');
//    Serial.print(rightMotorSpeed);
//    Serial.println();
//  }
}

/* Auto-calibration: turn right and left while calibrating the sensors.
 * baseDelayTime: base delay time, delayCounter: the number of loops to delay
 */
void init_calibrate(int baseDelayTime, int delayCounter) {
  for (int counter=0; counter<delayCounter; counter++) {
//    if (counter < 20 || counter >= 60)
//      set_motors(CALI_SPEED,-CALI_SPEED);
//    else
//      set_motors(-CALI_SPEED,CALI_SPEED);

    // This function records a set of sensor readings and keeps
    // track of the minimum and maximum values encountered.  The
    // IR_EMITTERS_ON argument means that the IR LEDs will be
    // turned on during the reading, which is usually what you
    // want.
    qtra.calibrate();

    // the total delay is (# of loops)*(delay time/loop),
    // i.e., 100*20 = 2000 ms.
    delay(baseDelayTime);
  }
}
