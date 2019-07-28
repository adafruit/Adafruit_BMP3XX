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

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)
#define INT_PIN 2 // connect adafruit int pin to your board
//#define INTERRUPT

bool new_BMP_data = false;

Adafruit_BMP3XX bmp; // I2C
//Adafruit_BMP3XX bmp(BMP_CS); // hardware SPI
//Adafruit_BMP3XX bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

void sensorDataREady() {
  new_BMP_data = true;
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("BMP388 test");

  if (!bmp.begin())
  {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1)
      ;
  }

#ifdef INTERRUPT

  pinMode(INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INT_PIN), sensorDataREady, FALLING);

  bmp.setConfig(BMP3_OVERSAMPLING_16X, BMP3_OVERSAMPLING_2X, BMP3_IIR_FILTER_COEFF_7, BMP3_NORMAL_MODE, BMP3_ODR_25_HZ, true);
#else
  bmp.setConfig(BMP3_OVERSAMPLING_16X, BMP3_OVERSAMPLING_2X, BMP3_IIR_FILTER_COEFF_7, BMP3_FORCED_MODE);
  // recommended for drone application
  // bmp.setConfig(BMP3_OVERSAMPLING_8X, BMP3_NO_OVERSAMPLING,BMP3_IIR_FILTER_COEFF_1,BMP3_NORMAL_MODE, BMP3_ODR_25_HZ);
#endif
}

void loop()
{
#ifdef INTERRUPT
  if(new_BMP_data) {
    new_BMP_data = false;
    if (!bmp.performReading())
  {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("Temperature = ");
  Serial.print(bmp.temperature);
  Serial.println(" *C");
  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");
  Serial.println();
  }
#else
  if (!bmp.performReading())
  {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("Temperature = ");
  Serial.print(bmp.temperature);
  Serial.println(" *C");
  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");
  Serial.println();
  delay(2000);
#endif
}
