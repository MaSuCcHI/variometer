
/***************************************************************************
  This is a library for the BMP3XX temperature & pressure sensor

  Designed specifically to work with the Adafruit BMP388 Breakout
  ----> http://www.adafruit.com/products/3966

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Adafruit_Sensor.h>
#include <ArxContainer.h>
#include <SPI.h>
#include <TimedAction.h>
#include <Wire.h>

#include "Adafruit_BMP3XX.h"

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define BZ_PIN 8

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;

void update_press();
void update_bz_state();

int bz_interval=500;
int bz_frequency=263;
void update_bz_tone();

float pressure = 0;
std::deque<float> pressure_deque{0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

#define VARIOMERTER_INTERVAL 200
TimedAction variomerter_action = TimedAction(VARIOMERTER_INTERVAL, update_press);

TimedAction bz_state_action = TimedAction(100,update_bz_state);
TimedAction bz_tone_action = TimedAction(100,update_bz_tone);

void setup() {
  tone(BZ_PIN, 262, 600);
  Serial.begin(115200);
  while (!Serial);

  if (!bmp.begin_I2C()) {  // hardware I2C mode, can pass in address & alt Wire
                           // if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI
                           // mode if (! bmp.begin_SPI(BMP_CS, BMP_SCK,
                           // BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

void loop() {
  variomerter_action.check();
  bz_state_action.check();
  bz_tone_action.check();
  
  pressure_deque.pop_front();
  pressure_deque.push_back(bmp.pressure);
  float pressure_ave = 0;
  for(int i = 0; i<10; i++) {
    pressure_ave += pressure_deque[i];
  }
  pressure = pressure_ave/10;
}

unsigned long old_timestamp = 0;
float old_pressure = 0;
float vertical_speed = 0;

void update_press() {
  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }

  // 気圧差分　単位　Pa
  float pressure_diff = pressure - old_pressure;
  old_pressure = pressure;

  // 高さの差分　単位　cm
  float height_diff = pressure_diff * 10;
  
  // 時間差分　単位 ms
  unsigned long now_timestamp = millis();
  unsigned long time_diff = now_timestamp - old_timestamp; 
  old_timestamp = now_timestamp;
  
  // 速度　単位　cm/s
  vertical_speed =  -1 * height_diff * 1000  / time_diff;

}

void update_bz_state() { 
//
  int update_interval = 1000;
  
  if (vertical_speed > 1000) {
    //危険な上昇
    bz_frequency = 710;
    update_interval = 100;
  } else if (vertical_speed > 500) {
    // 急速な上昇
     bz_frequency = 510;
     update_interval = 300;
  } else if (vertical_speed > 200) {
    // 上昇
     bz_frequency = 410;
     update_interval = 500;
  } else if (vertical_speed > 100) {
    // 緩やかな上昇
     bz_frequency = 310;
     update_interval = 1000;
  } else if (vertical_speed > -50) {
    //高度維持
    bz_frequency = 270;
    update_interval = 1200;

  } else if (vertical_speed > -100) {
    //滑空
    bz_frequency = 1000;
    update_interval = 1000;
  } else if (vertical_speed > -200) {
    //下降
    bz_frequency = 200;
    update_interval = 1000;
  } else if (vertical_speed > -500) {
    //急降下
    bz_frequency = 170;
    update_interval = 500;
  } else {
    //落下
    bz_frequency = 130;
    update_interval = 300;
  }

  if(update_interval != bz_interval) {
    bz_tone_action.setInterval(update_interval);
    bz_interval = update_interval;
  }
}


void update_bz_tone() {
  Serial.print(vertical_speed);
  Serial.print("cm/s,");
  Serial.println(bz_frequency);
  if(bz_frequency==1000&&bz_interval==1000){
    //　滑空の時は音を流さらない
    noTone(BZ_PIN);
  }else{
    tone(BZ_PIN,bz_frequency, bz_interval/3);
  }
}