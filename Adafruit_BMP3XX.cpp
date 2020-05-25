/*!
 * @file Adafruit_BMP3XX.cpp
 *
 * @mainpage Adafruit BMP3XX temperature & barometric pressure sensor driver
 *
 * @section intro_sec Introduction
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
 * @section author Author
 *
 * Written by Ladyada for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "Adafruit_BMP3XX.h"
#include "Arduino.h"

//#define BMP3XX_DEBUG

///! These SPI pins must be global in order to work with underlying library
int8_t _BMP3_SoftwareSPI_MOSI; ///< Global SPI MOSI pin
int8_t _BMP3_SoftwareSPI_MISO; ///< Global SPI MISO pin
int8_t _BMP3_SoftwareSPI_SCK;  ///< Global SPI Clock pin
TwoWire *_BMP3_i2c;            ///< Global I2C interface pointer

// Our hardware interface functions
static int8_t i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data,
                        uint16_t len);
static int8_t i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data,
                       uint16_t len);
static int8_t spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data,
                       uint16_t len);
static int8_t spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data,
                        uint16_t len);
static uint8_t spi_transfer(uint8_t x);
static void delay_msec(uint32_t ms);

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Instantiates sensor with Hardware SPI or I2C.
    @param  cspin SPI chip select. If not passed in, I2C will be used
*/
/**************************************************************************/
Adafruit_BMP3XX::Adafruit_BMP3XX(int8_t cspin) : _cs(cspin), _meas_end(0) {
  _BMP3_SoftwareSPI_MOSI = -1;
  _BMP3_SoftwareSPI_MISO = -1;
  _BMP3_SoftwareSPI_SCK = -1;
  _filterEnabled = _tempOSEnabled = _presOSEnabled = false;
}

/**************************************************************************/
/*!
    @brief  Instantiates sensor with Software (bit-bang) SPI.
    @param  cspin SPI chip select
    @param  mosipin SPI MOSI (Data from microcontroller to sensor)
    @param  misopin SPI MISO (Data to microcontroller from sensor)
    @param  sckpin SPI Clock
*/
/**************************************************************************/
Adafruit_BMP3XX::Adafruit_BMP3XX(int8_t cspin, int8_t mosipin, int8_t misopin,
                                 int8_t sckpin)
    : _cs(cspin) {
  _BMP3_SoftwareSPI_MOSI = mosipin;
  _BMP3_SoftwareSPI_MISO = misopin;
  _BMP3_SoftwareSPI_SCK = sckpin;
  _filterEnabled = _tempOSEnabled = _presOSEnabled = false;
}

/**************************************************************************/
/*!
    @brief Initializes the sensor

    Hardware ss initialized, verifies it is in the I2C or SPI bus, then reads
    calibration data in preparation for sensor reads.

    @param  addr Optional parameter for the I2C address of BMP3. Default is 0x77
    @param  theWire Optional parameter for the I2C device we will use. Default
   is "Wire"
    @return True on sensor initialization success. False on failure.
*/
/**************************************************************************/
bool Adafruit_BMP3XX::begin(uint8_t addr, TwoWire *theWire) {
  _i2caddr = addr;

  if (_cs == -1) {
    // i2c
    _BMP3_i2c = theWire;
    _BMP3_i2c->begin();

    the_sensor.dev_id = addr;
    the_sensor.intf = BMP3_I2C_INTF;
    the_sensor.read = &i2c_read;
    the_sensor.write = &i2c_write;
  } else {
    digitalWrite(_cs, HIGH);
    pinMode(_cs, OUTPUT);

    if (_BMP3_SoftwareSPI_SCK == -1) {
      // hardware SPI
      SPI.begin();
    } else {
      // software SPI
      pinMode(_BMP3_SoftwareSPI_SCK, OUTPUT);
      pinMode(_BMP3_SoftwareSPI_MOSI, OUTPUT);
      pinMode(_BMP3_SoftwareSPI_MISO, INPUT);
    }

    the_sensor.dev_id = _cs;
    the_sensor.intf = BMP3_SPI_INTF;
    the_sensor.read = &spi_read;
    the_sensor.write = &spi_write;
  }

  the_sensor.delay_ms = delay_msec;

  int8_t rslt = BMP3_OK;
  rslt = bmp3_init(&the_sensor);
#ifdef BMP3XX_DEBUG
  Serial.print("Result: ");
  Serial.println(rslt);
#endif

  if (rslt != BMP3_OK)
    return false;

#ifdef BMP3XX_DEBUG
  Serial.print("T1 = ");
  Serial.println(the_sensor.calib_data.reg_calib_data.par_t1);
  Serial.print("T2 = ");
  Serial.println(the_sensor.calib_data.reg_calib_data.par_t2);
  Serial.print("T3 = ");
  Serial.println(the_sensor.calib_data.reg_calib_data.par_t3);
  Serial.print("P1 = ");
  Serial.println(the_sensor.calib_data.reg_calib_data.par_p1);
  Serial.print("P2 = ");
  Serial.println(the_sensor.calib_data.reg_calib_data.par_p2);
  Serial.print("P3 = ");
  Serial.println(the_sensor.calib_data.reg_calib_data.par_p3);
  Serial.print("P4 = ");
  Serial.println(the_sensor.calib_data.reg_calib_data.par_p4);
  Serial.print("P5 = ");
  Serial.println(the_sensor.calib_data.reg_calib_data.par_p5);
  Serial.print("P6 = ");
  Serial.println(the_sensor.calib_data.reg_calib_data.par_p6);
  Serial.print("P7 = ");
  Serial.println(the_sensor.calib_data.reg_calib_data.par_p7);
  Serial.print("P8 = ");
  Serial.println(the_sensor.calib_data.reg_calib_data.par_p8);
  Serial.print("P9 = ");
  Serial.println(the_sensor.calib_data.reg_calib_data.par_p9);
  Serial.print("P10 = ");
  Serial.println(the_sensor.calib_data.reg_calib_data.par_p10);
  Serial.print("P11 = ");
  Serial.println(the_sensor.calib_data.reg_calib_data.par_p11);
  // Serial.print("T lin = ");
  // Serial.println(the_sensor.calib_data.reg_calib_data.t_lin);
#endif

  setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
  setPressureOversampling(BMP3_NO_OVERSAMPLING);
  setIIRFilterCoeff(BMP3_IIR_FILTER_DISABLE);

  // don't do anything till we request a reading
  the_sensor.settings.op_mode = BMP3_FORCED_MODE;

  return true;
}

/**************************************************************************/
/*!
    @brief Performs a reading and returns the ambient temperature.
    @return Temperature in degrees Centigrade
*/
/**************************************************************************/
float Adafruit_BMP3XX::readTemperature(void) {
  performReading();
  return temperature;
}

/**************************************************************************/
/*!
    @brief Performs a reading and returns the barometric pressure.
    @return Barometic pressure in Pascals
*/
/**************************************************************************/
float Adafruit_BMP3XX::readPressure(void) {
  performReading();
  return pressure;
}

/**************************************************************************/
/*!
    @brief Calculates the altitude (in meters).

    Reads the current atmostpheric pressure (in hPa) from the sensor and
   calculates via the provided sea-level pressure (in hPa).

    @param  seaLevel      Sea-level pressure in hPa
    @return Altitude in meters
*/
/**************************************************************************/
float Adafruit_BMP3XX::readAltitude(float seaLevel) {
  // Equation taken from BMP180 datasheet (page 16):
  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude. See this thread for more information:
  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

  float atmospheric = readPressure() / 100.0F;
  return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
}

/**************************************************************************/
/*!
    @brief Performs a full reading of all sensors in the BMP3XX.

    Assigns the internal Adafruit_BMP3XX#temperature & Adafruit_BMP3XX#pressure
   member variables

    @return True on success, False on failure
*/
/**************************************************************************/
bool Adafruit_BMP3XX::performReading(void) {
  int8_t rslt;
  /* Used to select the settings user needs to change */
  uint16_t settings_sel = 0;
  /* Variable used to select the sensor component */
  uint8_t sensor_comp = 0;

  /* Select the pressure and temperature sensor to be enabled */
  the_sensor.settings.temp_en = BMP3_ENABLE;
  settings_sel |= BMP3_TEMP_EN_SEL;
  sensor_comp |= BMP3_TEMP;
  if (_tempOSEnabled) {
    settings_sel |= BMP3_TEMP_OS_SEL;
  }

  the_sensor.settings.press_en = BMP3_ENABLE;
  settings_sel |= BMP3_PRESS_EN_SEL;
  sensor_comp |= BMP3_PRESS;
  if (_presOSEnabled) {
    settings_sel |= BMP3_PRESS_OS_SEL;
  }

  if (_filterEnabled) {
    settings_sel |= BMP3_IIR_FILTER_SEL;
  }

  if (_ODREnabled) {
    settings_sel |= BMP3_ODR_SEL;
  }

  // set interrupt to data ready
  // settings_sel |= BMP3_DRDY_EN_SEL | BMP3_LEVEL_SEL | BMP3_LATCH_SEL;

  /* Set the desired sensor configuration */
#ifdef BMP3XX_DEBUG
  Serial.println("Setting sensor settings");
#endif
  rslt = bmp3_set_sensor_settings(settings_sel, &the_sensor);
  if (rslt != BMP3_OK)
    return false;

  /* Set the power mode */
  the_sensor.settings.op_mode = BMP3_FORCED_MODE;
#ifdef BMP3XX_DEBUG
  Serial.println("Setting power mode");
#endif
  rslt = bmp3_set_op_mode(&the_sensor);
  if (rslt != BMP3_OK)
    return false;

  /* Variable used to store the compensated data */
  struct bmp3_data data;

  /* Temperature and Pressure data are read and stored in the bmp3_data instance
   */
  rslt = bmp3_get_sensor_data(sensor_comp, &data, &the_sensor);

  /* Save the temperature and pressure data */
  temperature = data.temperature;
  pressure = data.pressure;
  if (rslt != BMP3_OK)
    return false;

  return true;
}

/**************************************************************************/
/*!
    @brief  Setter for Temperature oversampling
    @param  oversample Oversampling setting, can be BMP3_NO_OVERSAMPLING,
   BMP3_OVERSAMPLING_2X, BMP3_OVERSAMPLING_4X, BMP3_OVERSAMPLING_8X,
   BMP3_OVERSAMPLING_16X, BMP3_OVERSAMPLING_32X
    @return True on success, False on failure
*/
/**************************************************************************/

bool Adafruit_BMP3XX::setTemperatureOversampling(uint8_t oversample) {
  if (oversample > BMP3_OVERSAMPLING_32X)
    return false;

  the_sensor.settings.odr_filter.temp_os = oversample;

  if (oversample == BMP3_NO_OVERSAMPLING)
    _tempOSEnabled = false;
  else
    _tempOSEnabled = true;

  return true;
}

/**************************************************************************/
/*!
    @brief  Setter for Pressure oversampling
    @param  oversample Oversampling setting, can be BMP3_NO_OVERSAMPLING,
   BMP3_OVERSAMPLING_2X, BMP3_OVERSAMPLING_4X, BMP3_OVERSAMPLING_8X,
   BMP3_OVERSAMPLING_16X, BMP3_OVERSAMPLING_32X
    @return True on success, False on failure
*/
/**************************************************************************/
bool Adafruit_BMP3XX::setPressureOversampling(uint8_t oversample) {
  if (oversample > BMP3_OVERSAMPLING_32X)
    return false;

  the_sensor.settings.odr_filter.press_os = oversample;

  if (oversample == BMP3_NO_OVERSAMPLING)
    _presOSEnabled = false;
  else
    _presOSEnabled = true;

  return true;
}

/**************************************************************************/
/*!
    @brief  Setter for IIR filter coefficient
    @param filtercoeff Coefficient of the filter (in samples). Can be
   BMP3_IIR_FILTER_DISABLE (no filtering), BMP3_IIR_FILTER_COEFF_1,
   BMP3_IIR_FILTER_COEFF_3, BMP3_IIR_FILTER_COEFF_7, BMP3_IIR_FILTER_COEFF_15,
   BMP3_IIR_FILTER_COEFF_31, BMP3_IIR_FILTER_COEFF_63, BMP3_IIR_FILTER_COEFF_127
    @return True on success, False on failure

*/
/**************************************************************************/
bool Adafruit_BMP3XX::setIIRFilterCoeff(uint8_t filtercoeff) {
  if (filtercoeff > BMP3_IIR_FILTER_COEFF_127)
    return false;

  the_sensor.settings.odr_filter.iir_filter = filtercoeff;

  if (filtercoeff == BMP3_IIR_FILTER_DISABLE)
    _filterEnabled = false;
  else
    _filterEnabled = true;

  return true;
}

/**************************************************************************/
/*!
    @brief  Setter for output data rate (ODR)
    @param odr Sample rate in Hz. Can be BMP3_ODR_200_HZ, BMP3_ODR_100_HZ,
   BMP3_ODR_50_HZ, BMP3_ODR_25_HZ, BMP3_ODR_12_5_HZ, BMP3_ODR_6_25_HZ,
   BMP3_ODR_3_1_HZ, BMP3_ODR_1_5_HZ, BMP3_ODR_0_78_HZ, BMP3_ODR_0_39_HZ,
   BMP3_ODR_0_2_HZ, BMP3_ODR_0_1_HZ, BMP3_ODR_0_05_HZ, BMP3_ODR_0_02_HZ,
   BMP3_ODR_0_01_HZ, BMP3_ODR_0_006_HZ, BMP3_ODR_0_003_HZ, or BMP3_ODR_0_001_HZ
    @return True on success, False on failure

*/
/**************************************************************************/
bool Adafruit_BMP3XX::setOutputDataRate(uint8_t odr) {
  if (odr > BMP3_ODR_0_001_HZ)
    return false;

  the_sensor.settings.odr_filter.odr = odr;

  _ODREnabled = true;

  return true;
}

/**************************************************************************/
/*!
    @brief  Reads 8 bit values over I2C
*/
/**************************************************************************/
int8_t i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data,
                uint16_t len) {
#ifdef BMP3XX_DEBUG
  Serial.print("\tI2C $");
  Serial.print(reg_addr, HEX);
  Serial.print(" => ");
#endif

  _BMP3_i2c->beginTransmission((uint8_t)dev_id);
  _BMP3_i2c->write((uint8_t)reg_addr);
  _BMP3_i2c->endTransmission();
  if (len != _BMP3_i2c->requestFrom((uint8_t)dev_id, (byte)len)) {
#ifdef BMP3XX_DEBUG
    Serial.print("Failed to read ");
    Serial.print(len);
    Serial.print(" bytes from ");
    Serial.println(dev_id, HEX);
#endif
    return 1;
  }
  while (len--) {
    *reg_data = (uint8_t)_BMP3_i2c->read();
#ifdef BMP3XX_DEBUG
    Serial.print("0x");
    Serial.print(*reg_data, HEX);
    Serial.print(", ");
#endif
    reg_data++;
  }
#ifdef BMP3XX_DEBUG
  Serial.println("");
#endif
  return 0;
}

/**************************************************************************/
/*!
    @brief  Writes 8 bit values over I2C
*/
/**************************************************************************/
int8_t i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data,
                 uint16_t len) {
#ifdef BMP3XX_DEBUG
  Serial.print("\tI2C $");
  Serial.print(reg_addr, HEX);
  Serial.print(" <= ");
#endif
  _BMP3_i2c->beginTransmission((uint8_t)dev_id);
  _BMP3_i2c->write((uint8_t)reg_addr);
  while (len--) {
    _BMP3_i2c->write(*reg_data);
#ifdef BMP3XX_DEBUG
    Serial.print("0x");
    Serial.print(*reg_data, HEX);
    Serial.print(", ");
#endif
    reg_data++;
  }
  _BMP3_i2c->endTransmission();
#ifdef BMP3XX_DEBUG
  Serial.println("");
#endif
  return 0;
}

/**************************************************************************/
/*!
    @brief  Reads 8 bit values over SPI
*/
/**************************************************************************/
static int8_t spi_read(uint8_t cspin, uint8_t reg_addr, uint8_t *reg_data,
                       uint16_t len) {
#ifdef BMP3XX_DEBUG
  Serial.print("\tSPI $");
  Serial.print(reg_addr, HEX);
  Serial.print(" => ");
#endif

  // If hardware SPI we should use transactions!
  if (_BMP3_SoftwareSPI_SCK == -1) {
    SPI.beginTransaction(
        SPISettings(BMP3XX_DEFAULT_SPIFREQ, MSBFIRST, SPI_MODE0));
  }

  digitalWrite(cspin, LOW);

  spi_transfer(reg_addr | 0x80);

  while (len--) {
    *reg_data = spi_transfer(0x00);
#ifdef BMP3XX_DEBUG
    Serial.print("0x");
    Serial.print(*reg_data, HEX);
    Serial.print(", ");
#endif
    reg_data++;
  }

  digitalWrite(cspin, HIGH);

  if (_BMP3_SoftwareSPI_SCK == -1) {
    SPI.endTransaction();
  }

#ifdef BMP3XX_DEBUG
  Serial.println("");
#endif
  return 0;
}

/**************************************************************************/
/*!
    @brief  Writes 8 bit values over SPI
*/
/**************************************************************************/
static int8_t spi_write(uint8_t cspin, uint8_t reg_addr, uint8_t *reg_data,
                        uint16_t len) {
#ifdef BMP3XX_DEBUG
  Serial.print("\tSPI $");
  Serial.print(reg_addr, HEX);
  Serial.print(" <= ");
#endif

  // If hardware SPI we should use transactions!
  if (_BMP3_SoftwareSPI_SCK == -1) {
    SPI.beginTransaction(
        SPISettings(BMP3XX_DEFAULT_SPIFREQ, MSBFIRST, SPI_MODE0));
  }

  digitalWrite(cspin, LOW);

  spi_transfer(reg_addr);
  while (len--) {
    spi_transfer(*reg_data);
#ifdef BMP3XX_DEBUG
    Serial.print("0x");
    Serial.print(*reg_data, HEX);
    Serial.print(", ");
#endif
    reg_data++;
  }

  digitalWrite(cspin, HIGH);

  if (_BMP3_SoftwareSPI_SCK == -1) {
    SPI.endTransaction();
  }

#ifdef BMP3XX_DEBUG
  Serial.println("");
#endif
  return 0;
}

static uint8_t spi_transfer(uint8_t x) {
  if (_BMP3_SoftwareSPI_SCK == -1)
    return SPI.transfer(x);

  // software spi
  // Serial.println("Software SPI");
  uint8_t reply = 0;
  for (int i = 7; i >= 0; i--) {
    reply <<= 1;
    digitalWrite(_BMP3_SoftwareSPI_SCK, LOW);
    digitalWrite(_BMP3_SoftwareSPI_MOSI, x & (1 << i));
    digitalWrite(_BMP3_SoftwareSPI_SCK, HIGH);
    if (digitalRead(_BMP3_SoftwareSPI_MISO))
      reply |= 1;
  }
  return reply;
}

static void delay_msec(uint32_t ms) { delay(ms); }
