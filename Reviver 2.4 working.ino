#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <UTFTGLUE.h>  //use GLUE class and constructor
#include "MCUFRIEND_kbv.h"
#include "bitmap_plane.h"
#include "Adafruit_GFX.h"  // Hardware-specific library
#include <Wire.h>
#include <Adafruit_Sensor.h>
//#include <Adafruit_LSM303_U.h>
#include <Adafruit_HMC5883_U.h>


////////////////////////////////////////////LOGICAL OPERATIONS///////////////////////////////////////////////
MCUFRIEND_kbv tft;

#define DEG2RAD 0.0174532925
#define BLACK 0x0000
#define LIGHT_BLUE 0x0FFF
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF
#define GREY 0x8410
#define ORANGE 0xE880

RF24 radio(23, 25); // CE, CSN

const byte address[6] = "00012";


int x = 1;
int Set_Heading = 90;
double Final_Heading = 0;
int Compass_Heading = 0;
double Previous_Compass_Heading = 0;
double previous_x3 = 0;
double previous_y3 = 0;
int Orientation_Changed_STS = 1;
int Buzzer_Pin = 49;

void setup() {
  Serial.begin(115200);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();
  uint16_t ID = tft.readID();
  Serial.print(F("ID = 0x"));
  Serial.println(ID, HEX);
  tft.begin(ID);
}

void loop() {
  int headingt;
  if (radio.available()) {
    radio.read(&headingt, sizeof(headingt));
  }
  Serial.println(Compass_Heading);
  Compass_Heading = headingt;

  tft.setRotation(0);
  tft.drawRect(0, 0, 240, 320, WHITE);


  if (Compass_Heading >= 10 && Compass_Heading < 20) {

    if (Previous_Compass_Heading >= 10 && Previous_Compass_Heading < 20) {

    } else {
      Orientation_Changed_STS = 1;
    }

  } else if (Compass_Heading >= 20 && Compass_Heading < 45) {

    if (Previous_Compass_Heading >= 20 && Previous_Compass_Heading < 45) {

    } else {
      Orientation_Changed_STS = 1;
    }


  } else if (Compass_Heading >= 45 && Compass_Heading < 70) {

    if (Previous_Compass_Heading >= 45 && Previous_Compass_Heading < 70) {

    } else {
      Orientation_Changed_STS = 1;
    }


  } else if (Compass_Heading >= 70 && Compass_Heading < 80) {

    if (Previous_Compass_Heading >= 70 && Previous_Compass_Heading < 80) {

    } else {
      Orientation_Changed_STS = 1;
    }


  } else if (Compass_Heading >= 80 && Compass_Heading < 105) {

    if (Previous_Compass_Heading >= 80 && Previous_Compass_Heading < 105) {

    } else {
      Orientation_Changed_STS = 1;
    }


  } else if (Compass_Heading >= 105 && Compass_Heading < 135) {

    if (Previous_Compass_Heading >= 105 && Previous_Compass_Heading < 135) {

    } else {
      Orientation_Changed_STS = 1;
    }

  } else if (Compass_Heading >= 135 && Compass_Heading < 160) {

    if (Previous_Compass_Heading >= 135 && Previous_Compass_Heading < 160) {

    } else {
      Orientation_Changed_STS = 1;
    }


  } else if (Compass_Heading >= 160 && Compass_Heading < 175) {

    if (Previous_Compass_Heading >= 160 && Previous_Compass_Heading < 175) {

    } else {
      Orientation_Changed_STS = 1;
    }


  } else if (Compass_Heading >= 175 && Compass_Heading < 195) {

    if (Previous_Compass_Heading >= 175 && Previous_Compass_Heading < 195) {

    } else {
      Orientation_Changed_STS = 1;
    }


  } else if (Compass_Heading >= 195 && Compass_Heading < 220) {

    if (Previous_Compass_Heading >= 195 && Previous_Compass_Heading < 220) {

    } else {
      Orientation_Changed_STS = 1;
    }


  } else if (Compass_Heading >= 220 && Compass_Heading < 230) {

    if (Previous_Compass_Heading >= 220 && Previous_Compass_Heading < 230) {

    } else {
      Orientation_Changed_STS = 1;
    }


  } else if (Compass_Heading >= 230 && Compass_Heading < 255) {

    if (Previous_Compass_Heading >= 230 && Previous_Compass_Heading < 255) {

    } else {
      Orientation_Changed_STS = 1;
    }


  } else if (Compass_Heading >= 255 && Compass_Heading < 275) {

    if (Previous_Compass_Heading >= 255 && Previous_Compass_Heading < 275) {

    } else {
      Orientation_Changed_STS = 1;
    }


  } else if (Compass_Heading >= 275 && Compass_Heading < 290) {

    if (Previous_Compass_Heading >= 275 && Previous_Compass_Heading < 290) {

    } else {
      Orientation_Changed_STS = 1;
    }


  } else if (Compass_Heading >= 290 && Compass_Heading < 315) {

    if (Previous_Compass_Heading >= 290 && Previous_Compass_Heading < 315) {

    } else {
      Orientation_Changed_STS = 1;
    }


  } else if (Compass_Heading >= 315 && Compass_Heading < 340) {

    if (Previous_Compass_Heading >= 315 && Previous_Compass_Heading < 340) {

    } else {
      Orientation_Changed_STS = 1;
    }


  } else if (Compass_Heading >= 340 && Compass_Heading < 350) {

    if (Previous_Compass_Heading >= 340 && Previous_Compass_Heading < 350) {

    } else {
      Orientation_Changed_STS = 1;
    }


  } else {

    if (Previous_Compass_Heading >= 350 || Previous_Compass_Heading < 10) {

    } else {
      Orientation_Changed_STS = 1;
    }
  }



  if (Orientation_Changed_STS == 1) {
    tft.fillScreen(BLACK);
  }

  int x5 = 40;
  int y5 = 80;
  int w5 = 160;

if (Compass_Heading >= 10 && Compass_Heading < 20) {

    if (Orientation_Changed_STS == 1) {
      tft.drawBitmap(x5, y5, image_data_Image6, w5, w5, WHITE);  // n-3
    }

  } else if (Compass_Heading >= 20 && Compass_Heading < 45) {
    if (Orientation_Changed_STS == 1) {
      tft.drawBitmap(x5, y5, image_data_Image5, w5, w5, WHITE);  // 3
    }

  } else if (Compass_Heading >= 45 && Compass_Heading < 70) {
    if (Orientation_Changed_STS == 1) {
      tft.drawBitmap(x5, y5, image_data_Image3, w5, w5, WHITE);  //6
    }

  } else if (Compass_Heading >= 70 && Compass_Heading < 80) {
    if (Orientation_Changed_STS == 1) {
      tft.drawBitmap(x5, y5, image_data_Image2, w5, w5, WHITE);  // 6-E
    }

  } else if (Compass_Heading >= 80 && Compass_Heading < 105) {
    if (Orientation_Changed_STS == 1) {
      tft.drawBitmap(x5, y5, image_data_Image1, w5, w5, WHITE);  // EAST
      
    }

  } else if (Compass_Heading >= 105 && Compass_Heading < 135) {
    if (Orientation_Changed_STS == 1) {
      tft.drawBitmap(x5, y5, image_data_Image23, w5, w5, WHITE);  // 12
    }

  } else if (Compass_Heading >= 135 && Compass_Heading < 160) {
    if (Orientation_Changed_STS == 1) {
      tft.drawBitmap(x5, y5, image_data_Image21, w5, w5, WHITE);  //15
    
    }

  } else if (Compass_Heading >= 160 && Compass_Heading < 175) {
    if (Orientation_Changed_STS == 1) {
      tft.drawBitmap(x5, y5, image_data_Image20, w5, w5, WHITE);  // 15-S
    }

  } else if (Compass_Heading >= 175 && Compass_Heading < 195) {
    if (Orientation_Changed_STS == 1) {
      tft.drawBitmap(x5, y5, image_data_Image19, w5, w5, WHITE);  //SOUTH
      
    }

  } else if (Compass_Heading >= 195 && Compass_Heading < 220) {
    if (Orientation_Changed_STS == 1) {
      tft.drawBitmap(x5, y5, image_data_Image17, w5, w5, WHITE);  // 21
    }

  } else if (Compass_Heading >= 220 && Compass_Heading < 230) {
    if (Orientation_Changed_STS == 1) {
      tft.drawBitmap(x5, y5, image_data_Image16, w5, w5, WHITE);  // 21-24
    }

  } else if (Compass_Heading >= 230 && Compass_Heading < 255) {
    if (Orientation_Changed_STS == 1) {
      tft.drawBitmap(x5, y5, image_data_Image15, w5, w5, WHITE);  // 24
    }

  } 
   else if (Compass_Heading >= 255 && Compass_Heading < 275) {
    if (Orientation_Changed_STS == 1) {
      tft.drawBitmap(x5, y5, image_data_Image13, w5, w5, WHITE);  //WEST
      
    }

  } else if (Compass_Heading >= 275 && Compass_Heading < 290) {
    if (Orientation_Changed_STS == 1) {
      tft.drawBitmap(x5, y5, image_data_Image12, w5, w5, WHITE);  // w-30
    }

  } else if (Compass_Heading >= 290 && Compass_Heading < 315) {
    if (Orientation_Changed_STS == 1) {
      tft.drawBitmap(x5, y5, image_data_Image11, w5, w5, WHITE);  // 30
    }


  } else if (Compass_Heading >= 315 && Compass_Heading < 340) {
    if (Orientation_Changed_STS == 1) {
      tft.drawBitmap(x5, y5, image_data_Image9, w5, w5, WHITE);  // 33
    }

  } else if (Compass_Heading >= 340 && Compass_Heading < 350) {
    if (Orientation_Changed_STS == 1) {
      tft.drawBitmap(x5, y5, image_data_Image8, w5, w5, WHITE);  // 33-N
    }

  } else {
    if (Orientation_Changed_STS == 1) {
      tft.drawBitmap(x5, y5, image_data_Image7, w5, w5, WHITE);  //NORTH
    }
  }



  if (Compass_Heading >= 269 && Compass_Heading < 271) {//w
    digitalWrite(Buzzer_Pin, HIGH);
    delay (250);
    digitalWrite(Buzzer_Pin, LOW);
    delay (1000);

  } else if (Compass_Heading >= 89 && Compass_Heading < 91) {//e
    digitalWrite(Buzzer_Pin, HIGH);
    delay (250);
    digitalWrite(Buzzer_Pin, LOW);
    delay (1000);

  } else if (Compass_Heading >= 179 && Compass_Heading < 181) {//s
    digitalWrite(Buzzer_Pin, HIGH);
    delay (250);
    digitalWrite(Buzzer_Pin, LOW);
    delay (1000);
  } 
  else if(Compass_Heading > 359){
    digitalWrite(Buzzer_Pin, HIGH);
    delay (250);
    digitalWrite(Buzzer_Pin, LOW);
    delay (1000);
  }
  else if(Compass_Heading < 1){
    digitalWrite(Buzzer_Pin, HIGH);
    delay (250);
    digitalWrite(Buzzer_Pin, LOW);
    delay (1000);
  }
  else {
    digitalWrite(Buzzer_Pin, LOW);
  }



  tft.drawBitmap(90, 128, image_data_plane_Final, 60, 60, YELLOW);
  tft.drawLine(190, 160, 45, 160, MAGENTA);
  //tft.drawCircle(x, y, radius, color);
  //  tft.setTextColor(WHITE);  tft.setTextSize(1);
  // tft.println("Total:");
  //  tft.println(0.000001 * total);
  //tft.drawLine(x1, y1, x2, y2, color);

  int x6 = 120;
  int y6 = 160;


  /*tft.drawCircle(x6, y6, 90, WHITE);
  tft.drawCircle(x6, y6, 91, WHITE);
  tft.drawCircle(x6, y6, 92, WHITE);
  tft.drawCircle(x6, y6, 93, WHITE);
  tft.drawCircle(x6, y6, 94, WHITE);
  tft.drawCircle(x6, y6, 95, WHITE);

  tft.drawCircle(x6, y6, 100, WHITE);
  tft.drawCircle(x6, y6, 101, WHITE);
  tft.drawCircle(x6, y6, 102, WHITE);
  tft.drawCircle(x6, y6, 103, WHITE);
  tft.drawCircle(x6, y6, 104, WHITE);
  */
  //tft.drawCircle(x6, y6, 105, WHITE);

  tft.drawLine(120, 60, 120, 80, WHITE);
  tft.drawLine(120, 240, 120, 260, WHITE);

  //tft.drawLine(235, 160, 240, 160, WHITE);
  tft.drawLine(20, 160, 40, 160, WHITE);

  tft.setRotation(1);

  int x1 = 130;
  int y1 = 10;
  tft.fillRect(x1, y1, 65, 20, BLACK);
  tft.setCursor(x1+10, y1);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print(Compass_Heading);
  tft.println(char(247));
  tft.drawRect(x1 - 10, y1 - 10, 85, 30, WHITE);
  tft.setRotation(0);
  

  tft.fillCircle(previous_x3 + 120, previous_y3 + 160, 5, BLACK);

  tft.setRotation(1);
  int x2 = 5;
  int y2 = 220;
  //tft.fillRect(x2, y2, 65, 25, BLACK);
  tft.setCursor(x2, y2);
  tft.setTextColor(LIGHT_BLUE);
  tft.setTextSize(2);
  tft.print("Ground Station");
  //tft.println(char(247));
  //tft.drawRect(x2 - 20, y2 - 10, 75, 35, LIGHT_BLUE);
  //tft.fillCircle(10, 17, 5, LIGHT_BLUE);
  tft.setRotation(0);

  tft.setRotation(1);
  int x312 = 215;
  int y312 = 5;
  //tft.fillRect(x2, y2, 65, 25, BLACK);
  tft.setCursor(x312, y312);
  tft.setTextColor(RED);
  tft.setTextSize(2);
  tft.print("M.D.Comp");
  //tft.println(char(247));
  //tft.drawRect(x2 - 20, y2 - 10, 75, 35, LIGHT_BLUE);
  //tft.fillCircle(10, 17, 5, LIGHT_BLUE);
  tft.setRotation(0);

  Final_Heading = Set_Heading - Compass_Heading;


  double x3 = 90 * cos((Final_Heading * M_PI) / 180);
  double y3 = 90 * sin((Final_Heading * M_PI) / 180);



  //tft.fillCircle(x3 + 120, y3 + 160, 5, LIGHT_BLUE);


  previous_x3 = x3;
  previous_y3 = y3;


  tft.drawLine(198, 160, 210, 155, WHITE);
  tft.drawLine(198, 160, 210, 165, WHITE);

  Previous_Compass_Heading = Compass_Heading;
  Orientation_Changed_STS = 0;
}
