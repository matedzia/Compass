#include <Arduino.h>

#include "Talkie.h"
#include "Vocab_US_Large.h"

int Compass_Heading ;

Talkie voice;


void setup() {
  Serial.begin(115200);

}

void loop() {
  
  if (Serial.available()>0){
  int x = Serial.read();
  Compass_Heading = x;
  Serial.println(Compass_Heading);
  }

  if (Compass_Heading >= 268 && Compass_Heading < 272) {//w
    // digitalWrite(Buzzer_Pin, HIGH);
    // delay (250);
    // digitalWrite(Buzzer_Pin, LOW);
    voice.say(sp5_HEADING);
    voice.say(sp3_WEST);
    delay (500);

  } else if (Compass_Heading >= 88 && Compass_Heading < 92) {//e
    // digitalWrite(Buzzer_Pin, HIGH);
    // delay (250);
    // digitalWrite(Buzzer_Pin, LOW);
    voice.say(sp5_HEADING);
    voice.say(sp3_EAST);
    delay (500);

  } else if (Compass_Heading >= 178 && Compass_Heading < 182) {//s
    // digitalWrite(Buzzer_Pin, HIGH);
    // delay (250);
    // digitalWrite(Buzzer_Pin, LOW);
    voice.say(sp5_HEADING);
    voice.say(sp3_SOUTH);
    delay (500);
  } 
  else if(Compass_Heading > 358){
    // digitalWrite(Buzzer_Pin, HIGH);
    // delay (250);
    // digitalWrite(Buzzer_Pin, LOW);
    voice.say(sp5_HEADING);
    voice.say(sp3_NORTH);
    delay (500);
  }
  else if(Compass_Heading < 2){
    // digitalWrite(Buzzer_Pin, HIGH);
    // delay (250);
    // digitalWrite(Buzzer_Pin, LOW);
    voice.say(sp5_HEADING);
    voice.say(sp3_NORTH);
    delay (500);
  }
  else {
   // digitalWrite(Buzzer_Pin, LOW);
  }

}