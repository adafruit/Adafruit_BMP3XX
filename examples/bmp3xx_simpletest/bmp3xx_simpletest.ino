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

#define SEALEVELPRESSURE_PA (101325)

bool new_BMP_data = false;

Adafruit_BMP3XX bmp; // I2C
//Adafruit_BMP3XX bmp(BMP_CS); // hardware SPI
//Adafruit_BMP3XX bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

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


  bmp.setSensorForcedModeSettings(BMP3_OVERSAMPLING_16X, BMP3_OVERSAMPLING_2X, BMP3_IIR_FILTER_COEFF_7);
  // recommended for drone application
    // bmp.setSensorNormalModeSettings(BMP3_OVERSAMPLING_8X, BMP3_NO_OVERSAMPLING,BMP3_IIR_FILTER_COEFF_1, BMP3_ODR_50_HZ);
  // put the sensor in sleep mode
    // bmp.setSensorInSleepMode();
}

void loop()
{
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
  Serial.print(bmp.getAltitude(bmp.pressure, SEALEVELPRESSURE_PA));
  Serial.println(" m");
  Serial.println();
  delay(2000);
}
