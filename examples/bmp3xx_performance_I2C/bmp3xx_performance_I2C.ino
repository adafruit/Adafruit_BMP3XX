/***************************************************************************
  This is a library for the BMP3XX temperature & pressure sensor

  Designed specifically to work with the Adafruit BMP388 Breakout
  ----> http://www.adafruit.com/products/3966

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Rob TIllaart based upon work written by
  Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;

uint32_t start, stop;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Adafruit BMP388 / BMP390 performance measurement");

  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
    //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode
    //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  Serial.println();
  for (uint32_t speed = 50000; speed < 850000; speed += 50000)
  {
    Serial.print("I2C: \t");
    Serial.println(speed);
    Wire.setClock(speed);
    measure();
    delay(2000);
  }

  Serial.println("\ndone...");
}

void loop() {
}


void measure() {
  Serial.println();
  delay(100);
  start = micros();
  bmp.performReading();
  stop = micros();
  Serial.print("performReading:\t");
  Serial.println(stop - start);
  delay(100);

  start = micros();
  float temperature = bmp.readTemperature();
  stop = micros();
  Serial.print("readTemperature:");
  Serial.print(stop - start);
  Serial.print("\t");
  Serial.println(temperature);
  delay(100);

  start = micros();
  float pressure = bmp.readPressure();
  stop = micros();
  Serial.print("readPressure:\t");
  Serial.print(stop - start);
  Serial.print("\t");
  Serial.println(pressure * 0.01  );
  delay(100);

  start = micros();
  float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  stop = micros();
  Serial.print("readAltitude:\t");
  Serial.print(stop - start);
  Serial.print("\t");
  Serial.println(altitude);
  delay(100);
  Serial.println();
}