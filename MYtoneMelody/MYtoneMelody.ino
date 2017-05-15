/*
  Arduino Mario Bros Tunes
  With Piezo Buzzer and PWM
 
  Connect the positive side of the Buzzer to pin 3,
  then the negative side to a 1k ohm resistor. Connect
  the other side of the 1 k ohm resistor to
  ground(GND) pin on the Arduino.
 
  by: Dipto Pratyaksa
  last updated: 31/3/13
*/
#include "pitches.h"

int castleSky[] = {
  0, 0, 0, NOTE_A4, NOTE_B4, 
  NOTE_C5, NOTE_B4, NOTE_C5, NOTE_E5, 
  NOTE_B4, 0, NOTE_E4, 
  NOTE_A4, NOTE_G4, NOTE_A4, NOTE_C5, 
  NOTE_G4, 0, NOTE_E4, 
  NOTE_F4, NOTE_E4, NOTE_F4, NOTE_C5, 
  NOTE_E4, 0, NOTE_C5, NOTE_C5, NOTE_C5, 
  NOTE_B4, NOTE_FS4, NOTE_FS4, NOTE_B4, 
  NOTE_B4, 0, NOTE_A4, NOTE_B4,
  NOTE_C5, NOTE_B4, NOTE_C5, NOTE_E5,
  NOTE_B4, 0, 0, NOTE_E4,
  // REPEAT
  NOTE_A4, NOTE_G4, NOTE_A4, NOTE_C5, 
  NOTE_G4, 0, NOTE_E4,
  NOTE_F4, NOTE_C5, NOTE_B4, NOTE_C5,
  NOTE_D5, NOTE_D5, NOTE_E5, NOTE_C5, 0,
  NOTE_C5, NOTE_B4, NOTE_A4, NOTE_A4, NOTE_B4, NOTE_GS4,
  NOTE_A4, 0, NOTE_C5, NOTE_D5,
  NOTE_E5, NOTE_D5, NOTE_E5, NOTE_G5,
  NOTE_D5, 0, NOTE_E4, NOTE_E4,
  NOTE_C5, NOTE_B4, NOTE_C5, NOTE_E5,
  NOTE_E5, 0, 0,
  NOTE_A4, NOTE_B4, NOTE_C5, NOTE_B4, NOTE_D5,
  NOTE_C5, NOTE_G4, NOTE_G4, 0,
  NOTE_F5, NOTE_E5, NOTE_D5, NOTE_C5,
  NOTE_E5
};

int tempoCastleSky[] = {
  4, 4, 4, 8, 8, 
  3, 6, 4, 4,
  2, 4, 4,
  3, 6, 8, 2.6,
  2, 4, 4, 
  3, 6, 4, 4, 
  2, 8, 8, 8, 8,
  3, 6, 4, 4,
  2, 4, 8, 8,
  3, 6, 4, 4,
  2, 4, 8, 8,
  //repeat
  3, 6, 4, 4,
  2, 4, 4,
  4, 8, 2.6, 4,
  6, 6, 6, 4, 4,
  8, 8, 8, 8, 4, 4,
  8, 2.6, 4, 4,
  3, 6, 4, 4,
  2, 4, 8, 8,
  3, 6, 4, 4,
  2, 4, 4,
  8, 8, 4, 4, 4, 
  3, 6, 4, 4,
  4, 4, 4, 4,
  1
};

int flyMeToTheMoon[] = {
  NOTE_C5, NOTE_B4, NOTE_A4, NOTE_G4, NOTE_F4,
  NOTE_G4, NOTE_A4, NOTE_C5, NOTE_B4, 
  NOTE_A4, NOTE_G4, NOTE_F4, NOTE_E4, 
  0, 0, 0,
  NOTE_A4, NOTE_G4, NOTE_F4, NOTE_E4, NOTE_D4,
  NOTE_E4, NOTE_F4, NOTE_A4, 
  NOTE_GS4, NOTE_F4, NOTE_E4, NOTE_D4, NOTE_C4,
  // Ending
  NOTE_CS4, NOTE_D4, NOTE_A4, NOTE_A4,
  NOTE_C5, NOTE_B4,
  NOTE_E5,
  0, 0, 0, NOTE_E5,
  NOTE_E5, NOTE_C5, NOTE_C5,
  0, 0, NOTE_C5, NOTE_D5,
  NOTE_C5, 0, 0, 0
//    ,
//  NOTE_CS4, NOTE_D4, NOTE_A4, NOTE_A4, 0, NOTE_C5, NOTE_B4, NOTE_G4,
//  0, NOTE_B3, NOTE_C4, NOTE_F4, NOTE_F4, 0, NOTE_A4, NOTE_GS4, NOTE_F4, NOTE_E4
};

int tempoFlyMe[] = {
  3, 6, 8, 4, 2.3,
  6, 8, 4, 2.3,
  6, 8, 4, 2.3,
  4, 4, 4,
  3, 6, 8, 4, 2.3,
  6, 4, 4,
  4, 4, 6, 4, 2.3,
  2, 3, 4, 1.8,
  4, 3,
  1,
  4, 4, 4, 4,
  4, 4, 2,
  4, 4, 4, 1.8,
  4, 4, 4, 4,
  1, 4, 4, 4
  
};

int ledPin = 13;
int speakerOut = 9;
int statePin = LOW;

void setup() {
  pinMode(speakerOut, OUTPUT);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  sing(1);
  delay(2000);
  sing(2);
  delay(2000);
}

int song = 0;
int tempoFactor = 1.30;

void sing(int s) {
  int size = 0;
  switch(s) {
    case 1:
      Serial.println("Castle of the Sky");
      size = sizeof(castleSky) / sizeof(int);
      for (int thisNote = 0; thisNote < size; thisNote++) {
   
        // to calculate the note duration, take one second
        // divided by the note type.
        //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
        int noteDuration = 1000 / tempoCastleSky[thisNote];
   
        buzz(speakerOut, castleSky[thisNote], noteDuration);
   
        // to distinguish the notes, set a minimum time between them.
        // the note's duration + 30% seems to work well:
        int pauseBetweenNotes = noteDuration * tempoFactor;
        delay(pauseBetweenNotes);
   
        // stop the tone playing:
        buzz(speakerOut, 0, noteDuration);
      }
      break;
    case 2:
      Serial.println("Fly me to the moon");
      size = sizeof(flyMeToTheMoon) / sizeof(int);
      for (int thisNote = 0; thisNote < size; thisNote++) {
   
        // to calculate the note duration, take one second
        // divided by the note type.
        //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
        int noteDuration = 1000 / tempoFlyMe[thisNote];
   
        buzz(speakerOut, flyMeToTheMoon[thisNote], noteDuration);
   
        // to distinguish the notes, set a minimum time between them.
        // the note's duration + 30% seems to work well:
        int pauseBetweenNotes = noteDuration * tempoFactor;
        delay(pauseBetweenNotes);
   
        // stop the tone playing:
        buzz(speakerOut, 0, noteDuration);
      }
      break;
    default:
    break;
  }
}

void buzz(int targetPin, long frequency, long length) {
  digitalWrite(ledPin, HIGH);
  long delayValue = 1000000 / frequency / 2; // calculate the delay value between transitions
  //// 1 second's worth of microseconds, divided by the frequency, then split in half since
  //// there are two phases to each cycle
  long numCycles = frequency * length / 1000; // calculate the number of cycles for proper timing
  //// multiply frequency, which is really cycles per second, by the number of seconds to
  //// get the total number of cycles to produce
  for (long i = 0; i < numCycles; i++) { // for the calculated length of time...
    digitalWrite(targetPin, HIGH); // write the buzzer pin high to push out the diaphram
    delayMicroseconds(delayValue); // wait for the calculated delay value
    digitalWrite(targetPin, LOW); // write the buzzer pin low to pull back the diaphram
    delayMicroseconds(delayValue); // wait again or the calculated delay value
  }
  digitalWrite(ledPin, LOW);
 
}
