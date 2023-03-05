////////////////////////////////////////////LIBRARIES///////////////////////////////////////////////
#include <UTFTGLUE.h>  //use GLUE class and constructor
#include "MCUFRIEND_kbv.h"
#include "bitmap_plane.h"
#include "Adafruit_GFX.h"  // Hardware-specific library
#include <Wire.h>
#include <Adafruit_Sensor.h>
//#include <Adafruit_LSM303_U.h>
#include <Adafruit_LSM303_Accel.h>
//#include <Adafruit_HMC5883_U.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <ezButton.h>
#include "RTClib.h"
#include <Adafruit_LIS2MDL.h>
#include <Arduino.h>
#include "Talkie.h"
#include "Vocab_US_Large.h"
#include <math.h>

Talkie voice;


RTC_DS1307 rtc;
////////////////////////////////////////////CONNECTIONS///////////////////////////////////////////////
int Potentiometer_Pin = A15;
int Pot_pin = A8;
int Buzzer_Pin = 34;
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

ezButton toggleSwitch(24);

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

int x = 1;
int Set_Heading = 90;
double Final_Heading = 0;
int Compass_Heading = 0;
double Previous_Compass_Heading = 0;
double previous_x3 = 0;
double previous_y3 = 0;
int flying = 0;

unsigned long previousMillis = 0;   // will store last time LED was updated
unsigned long previousMillis2 = 0;  // will store last time LED was updated
unsigned long previousMillis3 = 0;  // will store last time LED was updated
unsigned long previousMillis4 = 0;  // will store last time LED was updated

int Orientation_Changed_STS = 1;


int Potentiometer_ADC = 0;

int Pot_ADC = 0;


/* Assign a unique ID to this sensor at the same time */
//Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(31201);
//Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(31201);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LIS2MDL lis2mdl = Adafruit_LIS2MDL(12345);

template <typename T> struct vector
{
  T x, y, z;
};

// Stores min and max magnetometer values from calibration
vector<float> m_max;
vector<float> m_min;

/*
 * Returns the angular difference in the horizontal plane between the "from" vector and north, in degrees.
 * Description of heading algorithm:
 * Shift and scale the magnetic reading based on calibration data to find
 * the North vector. Use the acceleration readings to determine the Up
 * vector (gravity is measured as an upward acceleration). The cross
 * product of North and Up vectors is East. The vectors East and North
 * form a basis for the horizontal plane. The From vector is projected
 * into the horizontal plane and the angle between the projected vector
 * and horizontal north is returned.
 */
template <typename T> float heading(vector<T> from)
{
  sensors_event_t event;
   lis2mdl.getEvent(&event);
  //mag.getEvent(&event);
  vector<float> temp_m = {event.magnetic.x, event.magnetic.y, event.magnetic.z};

  accel.getEvent(&event);
  vector<float> a = {event.acceleration.x, event.acceleration.y, event.acceleration.z};
  
  // Important: subtract average of min and max from magnetometer calibration
  temp_m.x -= (m_min.x + m_max.x) / 2;
  temp_m.y -= (m_min.y + m_max.y) / 2;
  temp_m.z -= (m_min.z + m_max.z) / 2;

  // Compute east and north vectors
  vector<float> east;
  vector<float> north;
  vector_cross(&temp_m, &a, &east);
  vector_normalize(&east);
  vector_cross(&a, &east, &north);
  vector_normalize(&north);

  // compute heading
  int heading = atan2(vector_dot(&east, &from), vector_dot(&north, &from)) * 180 / PI;
  if (heading < 0) {
    heading += 360;
  }
  return heading;
}

template <typename Ta, typename Tb, typename To> void vector_cross(const vector<Ta> *a, const vector<Tb> *b, vector<To> *out)
{
  out->x = (a->y * b->z) - (a->z * b->y);
  out->y = (a->z * b->x) - (a->x * b->z);
  out->z = (a->x * b->y) - (a->y * b->x);
}

template <typename Ta, typename Tb> float vector_dot(const vector<Ta> *a, const vector<Tb> *b)
{
  return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}

void vector_normalize(vector<float> *a)
{
  float mag = sqrt(vector_dot(a, a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}



void setup() {
  Serial.begin(115200);
    Serial1.begin(115200);
  Serial.println(F("Arduino MEGA Setup Stage"));
  Potentiometer_Setup_Stage();
  TFT_Shield_Setup_Stage();
  Buzzer_Setup_Stage();
  LSM303_Setup_Stage();
  Radio_setup();
  Clk_setup();
  
}

void loop() {

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= 500) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    TFT_Shield_Loop_Stage();
  }

  if (currentMillis - previousMillis2 >= 250) {
    // save the last time you blinked the LED
    previousMillis2 = currentMillis;
    Logical_Operations();
  }

  if (currentMillis - previousMillis3 >= 250) {
    // save the last time you blinked the LED
    previousMillis3 = currentMillis;
    LSM303_Loop_Stage();
  } 
}

void Clk_setup(){
#ifndef ESP8266
  while (!Serial); // wait for serial port to connect. Needed for native USB
#endif

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }

  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running, let's set the time!");
    //FIRST TIME USE
     rtc.adjust(DateTime(2023, 2, 15, 13, 31, 30));
  }

  // RESET
    rtc.adjust(DateTime(2023, 3, 3, 23, 59, 59));  
}

void Radio_setup(){
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);
  radio.stopListening();
  }

void LSM303_Setup_Stage() {
  Serial.println("Compass Test\n");

  // Values determined with calibration example and lots of LSM303 wiggling
  m_min = (vector<float>){-81.90, -46.65, -68.85};
  m_max = (vector<float>){0.00, 47.40, 4.65};
  
  if (!lis2mdl.begin())
  {
    Serial.println("Unable to initialize LSM303 magnetometer");
    while (1);
  }

  if (!accel.begin()) {
    Serial.println("Unable to initialize LSM303 accelerometer");
    while (1);
  }

  accel.setRange(LSM303_RANGE_4G);
  accel.setMode(LSM303_MODE_NORMAL);
}


void displaySensorDetails(void) {
  // sensor_t sensor;
  // mag.getSensor(&sensor);

  //    sensors_event_t event;
  // lis2mdl.getEvent(&event);
  // Serial.println("------------------------------------");
  // Serial.print("Sensor:       ");
  // Serial.println(sensor.name);
  // Serial.print("Driver Ver:   ");
  // Serial.println(sensor.version);
  // Serial.print("Unique ID:    ");
  // Serial.println(sensor.sensor_id);
  // Serial.print("Max Value:    ");
  // Serial.print(sensor.max_value);
  // Serial.println(" uT");
  // Serial.print("Min Value:    ");
  // Serial.print(sensor.min_value);
  // Serial.println(" uT");
  // Serial.print("Resolution:   ");
  // Serial.print(sensor.resolution);
  // Serial.println(" uT");
  // Serial.println("------------------------------------");
  // Serial.println("");
  // delay(500);
}

/*
 * Returns the angular difference in the horizontal plane between a default vector and north, in degrees.
 * The default vector here is the +X axis as indicated by the silkscreen.
 */
int heading(void)
{
  return heading((vector<int>) {1, 0, 0});
}

void LSM303_Loop_Stage() {
  // /* Get a new sensor event */
  // // sensors_event_t event;
  // // mag.getEvent(&event);
  //  sensors_event_t event;
  // lis2mdl.getEvent(&event);


  // /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  // // Serial.print("X: ");
  // // Serial.print(event.magnetic.x);
  // // Serial.print("  ");
  // // Serial.print("Y: ");
  // // Serial.print(event.magnetic.y);
  // // Serial.print("  ");
  // // Serial.print("Z: ");
  // // Serial.print(event.magnetic.z);
  // // Serial.print("  ");
  // // Serial.println("uT");


  // float Pi = 3.14159;

  // // Calculate the angle of the vector y,x
  // float heading = (atan2(event.magnetic.y, event.magnetic.x) * 180) / Pi;

  // // Normalize to 0-360
  // if (heading < 0) {
  //   heading = 360 + heading;
  // }
  // Serial.print(" Heading: ");
  // Serial.print(heading);
  Compass_Heading = heading();
  // Wirelessly send Heading Value to second Arduino
  radio.write(&Compass_Heading, sizeof(Compass_Heading));
  Serial1.write(Compass_Heading);
  
}


void Potentiometer_Setup_Stage() {
  pinMode(Potentiometer_Pin, INPUT);
  pinMode(Pot_pin, INPUT);
}

void Buzzer_Setup_Stage() {
  pinMode(Buzzer_Pin, OUTPUT);
}

void TFT_Shield_Setup_Stage() {
  uint16_t ID = tft.readID();
  Serial.print(F("ID = 0x"));
  Serial.println(ID, HEX);
  tft.begin(ID);
}


void Logical_Operations() {

  Potentiometer_ADC = analogRead(Potentiometer_Pin);
  // Serial.print(F("Potentiometer_ADC = "));
  // Serial.println(Potentiometer_ADC);

  Set_Heading = map(Potentiometer_ADC, 0, 1023, 360, 0);

  // Serial.print(F("Set_Heading = "));
  // Serial.print(Set_Heading);

  Pot_ADC = analogRead(Pot_pin);
  flying = map(Pot_ADC, 0, 1023, 100, 0);  
  // Serial.print(F("Flying = "));
  // Serial.println(flying);
}


void TFT_Shield_Loop_Stage() {
  digitalWrite(Buzzer_Pin, LOW);
  
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


  // Buzzer activation pionts
  if (Compass_Heading >= 269 && Compass_Heading < 271) {//w
    // digitalWrite(Buzzer_Pin, HIGH);
    // delay (250);
    // digitalWrite(Buzzer_Pin, LOW);
    //voice.say(sp3_WEST);
    delay (100);

  } else if (Compass_Heading >= 89 && Compass_Heading < 91) {//e
    // digitalWrite(Buzzer_Pin, HIGH);
    // delay (250);
    // digitalWrite(Buzzer_Pin, LOW);
    //voice.say(sp3_EAST);
    delay (100);

  } else if (Compass_Heading >= 178 && Compass_Heading < 182) {//s
    // digitalWrite(Buzzer_Pin, HIGH);
    // delay (250);
    // digitalWrite(Buzzer_Pin, LOW);
    //voice.say(sp3_SOUTH);
    delay (100);
  } 
  else if(Compass_Heading > 359){
    // digitalWrite(Buzzer_Pin, HIGH);
    // delay (250);
    // digitalWrite(Buzzer_Pin, LOW);
    //voice.say(sp3_NORTH);
    delay (100);
  }
  else if(Compass_Heading < 1){
    // digitalWrite(Buzzer_Pin, HIGH);
    // delay (250);
    // digitalWrite(Buzzer_Pin, LOW);
    //voice.say(sp3_NORTH);
    delay (100);
  }
  else {
    digitalWrite(Buzzer_Pin, LOW);
  }



  tft.drawBitmap(90, 128, image_data_plane_Final, 60, 60, YELLOW);
  tft.drawLine(190, 160, 45, 160, MAGENTA);

  int x6 = 120;
  int y6 = 160;


  /*
  tft.drawCircle(x6, y6, 90, WHITE);
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
  
  tft.drawCircle(x6, y6, 105, WHITE);
  */

  tft.drawLine(120, 60, 120, 80, WHITE);
  tft.drawLine(120, 240, 120, 260, WHITE);
  tft.drawLine(20, 160, 40, 160, WHITE);

  tft.setRotation(1);

  int x1 = 130;
  int y1 = 10;
  tft.fillRect(x1, y1, 75, 20, BLACK);
  tft.setCursor(x1+10, y1);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print(Compass_Heading);
  tft.println(char(247));
  tft.drawRect(x1 - 10, y1 - 10, 85, 30, WHITE);
  tft.setRotation(0);
 

  tft.fillCircle(previous_x3 + 120, previous_y3 + 160, 5, BLACK);

  tft.setRotation(1);
  int x2 = 20;
  int y2 = 10;
  tft.fillRect(x2, y2, 65, 25, BLACK);
  tft.setCursor(x2, y2);
  tft.setTextColor(LIGHT_BLUE);
  tft.setTextSize(2);
  tft.print(Set_Heading);
  tft.println(char(247));
  tft.drawRect(x2 - 20, y2 - 10, 75, 35, LIGHT_BLUE);
  tft.fillCircle(10, 17, 5, LIGHT_BLUE);
  tft.setRotation(0);

  tft.setRotation(1);
  int x312 = 215;
  int y312 = 5;
  tft.setCursor(x312, y312);
  tft.setTextColor(RED);
  tft.setTextSize(2);
  tft.print("M.D.Comp");
  tft.setRotation(0);

  tft.setRotation(1);
  int x103 = 5;
  int y103 = 220;
  tft.setCursor(x103, y103);
  tft.setTextColor(LIGHT_BLUE);
  tft.setTextSize(2);
  tft.print("Aircraft");
  tft.setRotation(0);  

  Final_Heading = Set_Heading - Compass_Heading;
  

  double x3 = 90 * cos((Final_Heading * M_PI) / 180);
  double y3 = 90 * sin((Final_Heading * M_PI) / 180);



  tft.fillCircle(x3 + 120, y3 + 160, 5, LIGHT_BLUE);


  previous_x3 = x3;
  previous_y3 = y3;


  tft.drawLine(198, 160, 210, 155, WHITE);
  tft.drawLine(198, 160, 210, 165, WHITE);

  Previous_Compass_Heading = Compass_Heading;
  Orientation_Changed_STS = 0;


  if(flying > 1){
     tft.setRotation(1);
     tft.fillRect(135, 220, 180, 15, BLACK);
  tft.setCursor(160, 220);
  tft.setTextColor(LIGHT_BLUE);
  tft.setTextSize(2);
  tft.print("STUDENT PILOT");
  Serial.print("STUDENT PILOT"); 
  Serial.print(", Heading = ");
  Serial.print(Compass_Heading); 
  Serial.print(F(", HDG BUG = "));
  Serial.println(Set_Heading); 
  tft.setRotation(0); 
  } else{
    tft.setRotation(1);
    tft.fillRect(135, 220, 180, 15, BLACK);
  tft.setCursor(195, 220);
  tft.setTextColor(LIGHT_BLUE);
  tft.setTextSize(2);
  tft.print("INSTRUCTOR");
  Serial.print("INSTRUCTOR"); 
  Serial.print(", Heading = ");
  Serial.print(Compass_Heading); 
  Serial.print(F(", HDG BUG = "));
  Serial.println(Set_Heading); 
  tft.setRotation(0); 
  }

   DateTime now = rtc.now();

    // Serial.print(now.year(), DEC);
    // Serial.print('/');
    // Serial.print(now.month(), DEC);
    // Serial.print('/');
    // Serial.print(now.day(), DEC);
    // Serial.print(" (");
    // Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    // Serial.print(") ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.print(" - ");

  tft.setRotation(1);
  tft.fillRect(215, 25, 104, 15, BLACK);
  tft.setCursor(215, 25);
  tft.setTextColor(LIGHT_BLUE);
  tft.setTextSize(2);
  tft.print(now.hour(), DEC);
  tft.print(':');
  tft.print(now.minute(), DEC);
  tft.print(':');
  tft.print(now.second(), DEC);
  tft.setRotation(0); 
  
    delay(1000);
  
}
