
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

#define VARIOMERTER_INTERVAL 100
TimedAction variomerter_action = TimedAction(VARIOMERTER_INTERVAL, update_press);
TimedAction bz_action = TimedAction(100,update_bz_state);

TimedAction bz_tone_action = TimedAction(100000,update_bz_tone);

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
  bz_action.check();
  bz_tone_action.check();
  
  pressure_deque.pop_front();
  pressure_deque.push_back(bmp.pressure);
  float pressure_ave = 0;
  for(int i = 0; i<10; i++) {
    pressure_ave += pressure_deque[i];
  }
  pressure = pressure_ave/10;
}

float old_pressure = 0;
float vertical_speed = 0;
void update_press() {
  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  float pressure_diff = pressure - old_pressure;
  old_pressure = pressure;
  // Serial.print(pressure_diff);

  vertical_speed = pressure_diff * 10;

   Serial.print(vertical_speed);
  Serial.println("cm/s");
}

void update_bz_state() { 

  bz_frequency = random(98, 392);  
  bz_tone_action.setInterval(bz_interval);
}


void update_bz_tone() {
  // tone(BZ_PIN,bz_frequency, bz_interval/3);
}