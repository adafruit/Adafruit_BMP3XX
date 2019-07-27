{\rtf1\ansi\deff0\nouicompat{\fonttbl{\f0\fnil\fcharset0 Calibri;}}
{\colortbl ;\red0\green0\blue255;}
{\*\generator Riched20 10.0.18362}\viewkind4\uc1 
\pard\sa200\sl276\slmult1\f0\fs22\lang12 /***************************************************************************\par
  This is a library for the BMP3XX temperature & pressure sensor\par
\par
  Designed specifically to work with the Adafruit BMP388 Breakout\par
  ----> {{\field{\*\fldinst{HYPERLINK http://www.adafruit.com/products/3966 }}{\fldrslt{http://www.adafruit.com/products/3966\ul0\cf0}}}}\f0\fs22\par
\par
  These sensors use I2C or SPI to communicate, 2 or 4 pins are required\par
  to interface.\par
\par
  Adafruit invests time and resources providing this open source code,\par
  please support Adafruit and open-source hardware by purchasing products\par
  from Adafruit!\par
\par
  Written by Limor Fried & Kevin Townsend for Adafruit Industries.\par
  BSD license, all text above must be included in any redistribution\par
 ***************************************************************************/\par
\par
#include <Wire.h>\par
#include <SPI.h>\par
#include <Adafruit_Sensor.h>\par
#include "Adafruit_BMP3XX.h"\par
\par
#define BMP_SCK 13\par
#define BMP_MISO 12\par
#define BMP_MOSI 11\par
#define BMP_CS 10\par
\par
#define SEALEVELPRESSURE_HPA (1013.25)\par
\par
Adafruit_BMP3XX bmp; // I2C\par
//Adafruit_BMP3XX bmp(BMP_CS); // hardware SPI\par
//Adafruit_BMP3XX bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);\par
\par
void setup() \{\par
  Serial.begin(115200);\par
  while (!Serial);\par
  Serial.println("BMP388 test");\par
\par
  if (!bmp.begin()) \{\par
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");\par
    while (1);\par
  \}\par
\par
  // Set up oversampling and filter initialization\par
  //bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);\par
  //bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);\par
  //bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);\par
  //bmp.setOutputDataRate(BMP3_ODR_50_HZ);\par
\par
  bmp.setConfig(BMP3_OVERSAMPLING_8X, BMP3_OVERSAMPLING_4X, BMP3_IIR_FILTER_COEFF_3, BMP3_FORCED_MODE);\par
  // recommended for drone application\par
  // bmp.setConfig(BMP3_OVERSAMPLING_8X, BMP3_NO_OVERSAMPLING,BMP3_IIR_FILTER_COEFF_1,BMP3_NORMAL_MODE, BMP3_ODR_25_HZ);\par
\par
\}\par
\par
void loop() \{\par
  if (! bmp.performReading()) \{\par
    Serial.println("Failed to perform reading :(");\par
    return;\par
  \}\par
  Serial.print("Temperature = ");\par
  Serial.print(bmp.temperature);\par
  Serial.println(" *C");\par
\par
  Serial.print("Pressure = ");\par
  Serial.print(bmp.pressure / 100.0);\par
  Serial.println(" hPa");\par
\par
  Serial.print("Approx. Altitude = ");\par
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));\par
  Serial.println(" m");\par
\par
  Serial.println();\par
  delay(2000);\par
\}\par
}
 