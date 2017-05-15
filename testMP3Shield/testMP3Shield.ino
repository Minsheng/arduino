/*************************************************** 
  This is an example for the Adafruit VS1053 Codec Breakout

  Designed specifically to work with the Adafruit VS1053 Codec Breakout 
  ----> https://www.adafruit.com/products/1381

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/


#include <MemoryFree.h>


// include SPI, MP3 and SD libraries
#include <SPI.h>
#include <Adafruit_VS1053.h>
#include <SD.h>

// These are the pins used for the breakout example
#define BREAKOUT_RESET  9      // VS1053 reset pin (output)
#define BREAKOUT_CS     10     // VS1053 chip select pin (output)
#define BREAKOUT_DCS    8      // VS1053 Data/command select pin (output)
// These are the pins used for the music maker shield
#define SHIELD_RESET  -1      // VS1053 reset pin (unused!)
#define SHIELD_CS     7      // VS1053 chip select pin (output)
#define SHIELD_DCS    6      // VS1053 Data/command select pin (output)

// These are common pins between breakout and shield
#define CARDCS 4     // Card chip select pin
// DREQ should be an Int pin, see http://arduino.cc/en/Reference/attachInterrupt
#define DREQ 3       // VS1053 Data request, ideally an Interrupt pin

Adafruit_VS1053_FilePlayer musicPlayer = 
  // create breakout-example object!
  //Adafruit_VS1053_FilePlayer(BREAKOUT_RESET, BREAKOUT_CS, BREAKOUT_DCS, DREQ, CARDCS);
  // create shield-example object!
  Adafruit_VS1053_FilePlayer(SHIELD_RESET, SHIELD_CS, SHIELD_DCS, DREQ, CARDCS);



/*
 * PIR sensor tester
 */
 
int ledPin = 13;                // choose the pin for the LED
int inputPin = 2;               // choose the input pin (for PIR sensor) 
int pirState = LOW;             // we start, assuming no motion detected
int val = 0;                    // variable for reading the pin status
unsigned long detectTime;      //the time we started the app, compared ot millis
unsigned long resetTime;
//this array below is initialized once, randomly selected in the loop
char* myFiles[]={"track001.mp3", "track002.mp3", "track003.mp3",
"track004.mp3"};

int randFile; //random number variable
char MP3;  //variable for MP3 filename

//VARS
//the time we give the sensor to calibrate (10-60 secs according to the datasheet)
int calibrationTime = 10;  
 
void setup() {
        
  
  Serial.begin(9600);
  delay(50);
  Serial.println("Adafruit VS1053 Library Test");  

  resetTime = millis();  //set reset time to millis and start counting
  
  // initialise the music player
  if (! musicPlayer.begin()) { // initialise the music player
     Serial.println(F("Couldn't find VS1053, do you have the right pins defined?"));
     while (1);
  }
  Serial.println(F("VS1053 found")); 


  //musicPlayer.sineTest(0x44, 500);    // Make a tone to indicate VS1053 is working at startup

  pinMode(ledPin, OUTPUT);      // declare LED as output
  pinMode(inputPin, INPUT);     // declare sensor as input

 //give the sensor some time to calibrate
  Serial.print("calibrating sensor ");
  for(int i = 0; i < calibrationTime; i++){
    Serial.print(".");
    delay(1000);
    }
  Serial.println(" done");
  Serial.println("SENSOR ACTIVE");
  delay(50);  
 
  if (!SD.begin(CARDCS)) {
    Serial.println(F("SD failed, or not present"));
    while (1);  // don't do anything more
  }
  //Serial.println("SD OK!");
  
  // list files
  printDirectory(SD.open("/"), 0);
  
  //show the array of mp3 files
  int mp3ArrSize = 20;  //get the array size
  Serial.println("------------------");
  Serial.print("MP3 Array Size: ");
  Serial.println(mp3ArrSize);
  for (int arrelement = 0; arrelement < mp3ArrSize; arrelement++) {
    // turn the pin on:
    Serial.println(myFiles[arrelement]);             
  }
  
    
  // Set volume for left, right channels. lower numbers == louder volume!
  musicPlayer.setVolume(10,10);  
  /***** Two interrupt options! *******/ 
  // This option uses timer0, this means timer1 & t2 are not required
  // (so you can use 'em for Servos, etc) BUT millis() can lose time
  // since we're hitchhiking on top of the millis() tracker
  //musicPlayer.useInterrupt(VS1053_FILEPLAYER_TIMER0_INT);
  
  // This option uses a pin interrupt. No timers required! But DREQ
  // must be on an interrupt pin. For Uno/Duemilanove/Diecimilla
  // that's Digital #2 or #3
  // See http://arduino.cc/en/Reference/attachInterrupt for other pins
  // *** This method is preferred
  if (! musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT))
    Serial.println(F("DREQ pin is not an interrupt pin"));  
  
}
 
void loop(){
  val = digitalRead(inputPin);  // read input value
  Serial.println(val);  

  randFile = random(20);  //set random file value anything between 1 and 10
  //do random array thing here
   char* MP3 = myFiles[randFile];  //select one of the filenames from the array

  Serial.println(MP3);    //show us which MP3 we selected

  if (val == HIGH) {            // check if the input is HIGH
    digitalWrite(ledPin, HIGH);  // turn LED ON
      // we have just turned on
      Serial.println("Motion detected!");
      detectTime = millis();
      
      if (!musicPlayer.playingMusic==true){
        if (! musicPlayer.playFullFile("merry02.mp3")) {
          Serial.print("Could not open");
          Serial.println("merry02.mp3");
          musicPlayer.softReset();          
          while (1);
        }     
        delay(1000);      
        if (! musicPlayer.startPlayingFile(MP3)) {
          Serial.print("Could not open");
          Serial.println(MP3);
          while (1);
        }         
        Serial.print("Start Playing ");
        Serial.println(MP3);          
     
      } 

    }

//if its been more than 15 seconds since no movement checs about once each second, stop playing    
    if ( (millis() - detectTime) >= 15000){
        Serial.println("No motion for 15 seconds");
        
          musicPlayer.stopPlaying();  
          musicPlayer.softReset();
          delay(200);    
        digitalWrite(ledPin,LOW);
        val = 0;             // we start, assuming no motion detected        
        detectTime = 0;
    }
    Serial.print("******* freeMemory()=");
    Serial.println(freeMemory());

    if ( (millis() - resetTime) >= 1800000){

      void(* resetFunc)(void)=0; //declare reset function at address 0
      if (!musicPlayer.playingMusic==true){
          musicPlayer.stopPlaying();  
         delay(250);     
      }      
      musicPlayer.softReset();
      delay(500);
      resetFunc(); //call reset             
    }
  
  delay(1000);
}

/// File listing helper
void printDirectory(File dir, int numTabs) {
   while(true) {
     
     File entry =  dir.openNextFile();
     if (! entry) {
       // no more files
       //Serial.println("**nomorefiles**");
       break;
     }
     for (uint8_t i=0; i<numTabs; i++) {
       Serial.print('\t');
     }
     Serial.print(entry.name());
     if (entry.isDirectory()) {
       Serial.println("/");
       printDirectory(entry, numTabs+1);
     } else {
       // files have sizes, directories do not
       Serial.print("\t\t");
       Serial.println(entry.size(), DEC);
     }
     entry.close();
   }
}



