/*!
 * @file Adafruit_BMP3XX.h
 *
 * Adafruit BMP3XX temperature & barometric pressure sensor driver
 *
 * This is the documentation for Adafruit's BMP3XX driver for the
 * Arduino platform.  It is designed specifically to work with the
 * Adafruit BMP388 breakout: https://www.adafruit.com/products/3966
 *
 * These sensors use I2C or SPI to communicate
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Ladyada for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#ifndef __BMP3XX_H__
#define __BMP3XX_H__

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include <Wire.h>
#include <SPI.h>
#include "bmp3.h"

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define BMP3XX_DEFAULT_ADDRESS (0x77) ///< The default I2C address
/*=========================================================================*/
#define BMP3XX_DEFAULT_SPIFREQ (1000000) ///< The default SPI Clock speed

/** Adafruit_BMP3XX Class for both I2C and SPI usage.
 *  Wraps the Bosch library for Arduino usage
 */

class Adafruit_BMP3XX
{
public:
  Adafruit_BMP3XX(int8_t cspin = -1);
  Adafruit_BMP3XX(int8_t cspin, int8_t mosipin, int8_t misopin, int8_t sckpin);

  bool begin(uint8_t addr = BMP3XX_DEFAULT_ADDRESS, TwoWire *theWire = &Wire);
  float readTemperature(void);
  float readPressure(void);
  float getSeaLevelPressure(double atmospheriquePressure, double YourActualAltitude);
  float readAltitude(float seaLevel);

  bool setConfig(
      uint8_t PressureOversampling = BMP3_NO_OVERSAMPLING,
      uint8_t TemperatureOversampling = BMP3_NO_OVERSAMPLING,
      uint8_t IIRFilter = BMP3_IIR_FILTER_DISABLE,
      uint8_t PowerMode = BMP3_FORCED_MODE,
      uint8_t OutputDataRate = BMP3_ODR_200_HZ,
      bool AddInterrupt = false);

  /// Perform a reading in blocking mode
  bool performReading(void);

  /// Temperature (Celsius) assigned after calling performReading()
  double temperature;
  /// Pressure (Pascals) assigned after calling performReading()
  double pressure;

private:
  bool _forcedModeEnabled;
  uint8_t _i2caddr;
  int32_t _sensorID;
  int8_t _cs;
  unsigned long _meas_end;

  uint8_t spixfer(uint8_t x);

  struct bmp3_dev the_sensor;
};

#endif