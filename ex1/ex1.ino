//Creation&Computation Examples
//blink an LED with a timer

/*
dont start with bad habits using delay(), build a timer to control blinking

*/


int ledControlPin = 3;

int blinkRate = 500; //how often in milliseconds the led turns on/off

long lastChange; //the variable is our first example of memory. it holds the time of the last change so we can see when it should happen again. We use a long because the numbers get very large
bool ledState;  //Another example of memory. this will hold info on the current state of the led (either on / off) so we can know what it should on the switch
        //we will also be using a trick specific to boolean variables to simplify changing states

void setup()
{
  pinMode(ledControlPin, OUTPUT); //all digital pins 0-13 can be either input or output, so you must specify
 
  Serial.begin(9600); //turn on the serial port so we can print code updates in the console 
    
}



void loop()
{
  

 if(millis()-lastChange>=blinkRate)   //the millis() command tells us in milliseconds how long its been since the program started running
 {                    //the test is to subtract the time that it last changed from the current time and then
                      //see if that value is equal to or greater than the blinkRate variable.  if yes- change, if no - do nothing,,,wait
  ledState=!ledState;         //this is a little trick that only works with boolean variables. the ! symbol tells it to be the opposite of what it is now
   
    digitalWrite(ledControlPin, ledState);  //turn the LED on/off 
   
    lastChange = millis();        //save the time of this change in memory, so it knows when to change next
   
    Serial.println(lastChange);
   
 }
  
  
}
