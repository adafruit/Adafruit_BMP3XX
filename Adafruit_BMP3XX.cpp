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

#include "Arduino.h"
#include "Adafruit_BMP3XX.h"

#define BMP3XX_DEBUG

///! These SPI pins must be global in order to work with underlying library
int8_t _BMP3_SoftwareSPI_MOSI; ///< Global SPI MOSI pin
int8_t _BMP3_SoftwareSPI_MISO; ///< Global SPI MISO pin
int8_t _BMP3_SoftwareSPI_SCK;  ///< Global SPI Clock pin
TwoWire *_BMP3_i2c;            ///< Global I2C interface pointer

// Our hardware interface functions
static int8_t i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
static int8_t i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
static int8_t spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
static int8_t spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
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
Adafruit_BMP3XX::Adafruit_BMP3XX(int8_t cspin)
  : _cs(cspin)
  , _meas_end(0)
{
  _BMP3_SoftwareSPI_MOSI = -1;
  _BMP3_SoftwareSPI_MISO = -1;
  _BMP3_SoftwareSPI_SCK = -1;
  //_filterEnabled = _tempOSEnabled = _presOSEnabled = false;
  _forcedModeEnabled = false;
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
Adafruit_BMP3XX::Adafruit_BMP3XX(int8_t cspin, int8_t mosipin, int8_t misopin, int8_t sckpin)
  : _cs(cspin)
{
  _BMP3_SoftwareSPI_MOSI = mosipin;
  _BMP3_SoftwareSPI_MISO = misopin;
  _BMP3_SoftwareSPI_SCK = sckpin;
  //_filterEnabled = _tempOSEnabled = _presOSEnabled = false;
  _forcedModeEnabled = false;
}

/**************************************************************************/
/*!
    @brief Initializes the sensor

    Hardware ss initialized, verifies it is in the I2C or SPI bus, then reads
    calibration data in preparation for sensor reads.

    @param  addr Optional parameter for the I2C address of BMP3. Default is 0x77
    @param  theWire Optional parameter for the I2C device we will use. Default is "Wire"
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
  Serial.print("Result: "); Serial.println(rslt);
#endif

  if (rslt != BMP3_OK)
    return false;

#ifdef BMP3XX_DEBUG
  Serial.print("T1 = "); Serial.println(the_sensor.calib_data.reg_calib_data.par_t1);
  Serial.print("T2 = "); Serial.println(the_sensor.calib_data.reg_calib_data.par_t2);
  Serial.print("T3 = "); Serial.println(the_sensor.calib_data.reg_calib_data.par_t3);
  Serial.print("P1 = "); Serial.println(the_sensor.calib_data.reg_calib_data.par_p1);
  Serial.print("P2 = "); Serial.println(the_sensor.calib_data.reg_calib_data.par_p2);
  Serial.print("P3 = "); Serial.println(the_sensor.calib_data.reg_calib_data.par_p3);
  Serial.print("P4 = "); Serial.println(the_sensor.calib_data.reg_calib_data.par_p4);
  Serial.print("P5 = "); Serial.println(the_sensor.calib_data.reg_calib_data.par_p5);
  Serial.print("P6 = "); Serial.println(the_sensor.calib_data.reg_calib_data.par_p6);
  Serial.print("P7 = "); Serial.println(the_sensor.calib_data.reg_calib_data.par_p7);
  Serial.print("P8 = "); Serial.println(the_sensor.calib_data.reg_calib_data.par_p8);
  Serial.print("P9 = "); Serial.println(the_sensor.calib_data.reg_calib_data.par_p9);
  Serial.print("P10 = "); Serial.println(the_sensor.calib_data.reg_calib_data.par_p10);
  Serial.print("P11 = "); Serial.println(the_sensor.calib_data.reg_calib_data.par_p11);
  //Serial.print("T lin = "); Serial.println(the_sensor.calib_data.reg_calib_data.t_lin);
#endif

  return setSensorInSleepMode();
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

    Reads the current atmostpheric pressure (in Pa) from the sensor and calculates
    via the provided sea-level pressure (in hPa).

    @param  seaLevel      Sea-level pressure in Pa
    @return Altitude in meters
*/
/**************************************************************************/
float Adafruit_BMP3XX::readAltitude(float seaLevel)
{
    // Equation taken from BMP180 datasheet (page 16):
    //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

    // Note that using the equation from wikipedia can give bad results
    // at high altitude. See this thread for more information:
    //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

    float atmospheric = readPressure();
    return 44330.0f * (1.0f - pow(atmospheric / seaLevel, 0.1902949f));
}

/**************************************************************************/
/*!
    @brief Performs a full reading of all sensors in the BMP3XX.

    Assigns the internal Adafruit_BMP3XX#temperature & Adafruit_BMP3XX#pressure member variables

    @return True on success, False on failure
*/
/**************************************************************************/
bool Adafruit_BMP3XX::performReading(void) {
  int8_t rslt;
 
  /* Variable used to select the sensor component */
  uint8_t sensor_comp;
 
  /* if forced mode, set power mode of the sensor at each reading*/
  if (_forcedModeEnabled){
    rslt = bmp3_set_op_mode(&the_sensor);
    if (rslt != BMP3_OK){
      printf("error set power mode \n");
      if (rslt == BMP3_E_INVALID_ODR_OSR_SETTINGS)
        printf("BMP3_E_INVALID_ODR_OSR_SETTINGS \n");
      return false;  
    }
  }

  /* Variable used to store the compensated data */
  struct bmp3_data data;
 
  /* Sensor component selection */
  sensor_comp = BMP3_PRESS | BMP3_TEMP;
 
  /* Temperature and Pressure data are read and stored in the bmp3_data instance */
  rslt = bmp3_get_sensor_data(sensor_comp, &data, &the_sensor);
  if (rslt != BMP3_OK)
    return false;
 
  /* Save the temperature and pressure data */
  temperature = data.temperature;
  pressure = data.pressure;

  return true;
}

/**************************************************************************/
/*!
    @brief  Reads 8 bit values over I2C
*/
/**************************************************************************/
int8_t i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
#ifdef BMP3XX_DEBUG
  Serial.print("\tI2C $"); Serial.print(reg_addr, HEX); Serial.print(" => ");
#endif

  _BMP3_i2c->beginTransmission((uint8_t)dev_id);
  _BMP3_i2c->write((uint8_t)reg_addr);
  _BMP3_i2c->endTransmission();
  if (len != _BMP3_i2c->requestFrom((uint8_t)dev_id, (byte)len)) {
#ifdef BMP3XX_DEBUG
    Serial.print("Failed to read "); Serial.print(len); Serial.print(" bytes from "); Serial.println(dev_id, HEX);
#endif
    return 1;
  }
  while (len--) {
    *reg_data = (uint8_t)_BMP3_i2c->read();
#ifdef BMP3XX_DEBUG
    Serial.print("0x"); Serial.print(*reg_data, HEX); Serial.print(", ");
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
int8_t i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
#ifdef BMP3XX_DEBUG
  Serial.print("\tI2C $"); Serial.print(reg_addr, HEX); Serial.print(" <= ");
#endif
  _BMP3_i2c->beginTransmission((uint8_t)dev_id);
  _BMP3_i2c->write((uint8_t)reg_addr);
  while (len--) {
    _BMP3_i2c->write(*reg_data);
#ifdef BMP3XX_DEBUG
    Serial.print("0x"); Serial.print(*reg_data, HEX); Serial.print(", ");
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
static int8_t spi_read(uint8_t cspin, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
#ifdef BMP3XX_DEBUG
  Serial.print("\tSPI $"); Serial.print(reg_addr, HEX); Serial.print(" => ");
#endif

  // If hardware SPI we should use transactions!
  if (_BMP3_SoftwareSPI_SCK == -1) {
    SPI.beginTransaction(SPISettings(BMP3XX_DEFAULT_SPIFREQ, MSBFIRST, SPI_MODE0));
  }

  digitalWrite(cspin, LOW);

  spi_transfer(reg_addr | 0x80);

  while (len--) {
    *reg_data = spi_transfer(0x00);
#ifdef BMP3XX_DEBUG
    Serial.print("0x"); Serial.print(*reg_data, HEX); Serial.print(", ");
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
static int8_t spi_write(uint8_t cspin, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
#ifdef BMP3XX_DEBUG
  Serial.print("\tSPI $"); Serial.print(reg_addr, HEX); Serial.print(" <= ");
#endif

  // If hardware SPI we should use transactions!
  if (_BMP3_SoftwareSPI_SCK == -1) {
    SPI.beginTransaction(SPISettings(BMP3XX_DEFAULT_SPIFREQ, MSBFIRST, SPI_MODE0));
  }

  digitalWrite(cspin, LOW);

  spi_transfer(reg_addr);
  while (len--) {
    spi_transfer(*reg_data);
#ifdef BMP3XX_DEBUG
    Serial.print("0x"); Serial.print(*reg_data, HEX); Serial.print(", ");
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
  //Serial.println("Software SPI");
  uint8_t reply = 0;
  for (int i=7; i>=0; i--) {
    reply <<= 1;
    digitalWrite(_BMP3_SoftwareSPI_SCK, LOW);
    digitalWrite(_BMP3_SoftwareSPI_MOSI, x & (1<<i));
    digitalWrite(_BMP3_SoftwareSPI_SCK, HIGH);
    if (digitalRead(_BMP3_SoftwareSPI_MISO))
      reply |= 1;
  }
  return reply;
}


static void delay_msec(uint32_t ms){
  delay(ms);
}

bool Adafruit_BMP3XX::setSensorInSleepMode(){
  uint8_t rslt = BMP3_OK;
  _forcedModeEnabled = false;
  rslt = bmp3_soft_reset(&the_sensor);
  return rslt == BMP3_OK;
}

/**************************************************************************/
/*!
    @brief configure the sensor with desired settings.

    @param TemperatureOversampling        default BMP3_NO_OVERSAMPLING
    @param PressureOversampling           default BMP3_NO_OVERSAMPLING
    @param IIRFilter                      default BMP3_IIR_FILTER_DISABLE
    @param PowerMode                      default BMP3_FORCED_MODE
    @param OutputDataRate                 default BMP3_ODR_200_HZ, set if PowerMode = BMP3_NORMAL_MODE
    @param AddInterrupt                   default false

    @return True on success, False on failure
*/
/**************************************************************************/
bool Adafruit_BMP3XX::setConfig( uint8_t TemperatureOversampling, uint8_t PressureOversampling, uint8_t IIRFilter, uint8_t PowerMode, uint8_t OutputDataRate, bool DataReadyInterrupt)
{
  int8_t rslt = BMP3_OK;
  uint16_t settings_sel = 0;


  /* Select the pressure and temperature sensor to be enabled */
  the_sensor.settings.press_en = BMP3_ENABLE;
  the_sensor.settings.temp_en = BMP3_ENABLE;
  settings_sel |= BMP3_PRESS_EN_SEL | BMP3_TEMP_EN_SEL;


  /* set pressure settings */
  if (PressureOversampling > BMP3_OVERSAMPLING_32X)
  {
    printf("error pressure oversampling \n");
    return false;
  }
  the_sensor.settings.odr_filter.press_os = PressureOversampling;
  settings_sel |= BMP3_PRESS_OS_SEL;


  /* set temperature settings */
  if (TemperatureOversampling > BMP3_OVERSAMPLING_32X)
  {
    printf("error temp oversampling \n");
    return false;
  }
  the_sensor.settings.odr_filter.temp_os = TemperatureOversampling;
  settings_sel |= BMP3_TEMP_OS_SEL;

  /* set iirFilter settings */
  if (IIRFilter > BMP3_IIR_FILTER_COEFF_127)
  {
    printf("error iir filter \n");
    return false;
  }
  the_sensor.settings.odr_filter.iir_filter = IIRFilter;
  settings_sel |= BMP3_IIR_FILTER_SEL;

  /* set output data rate */
  if (PowerMode == BMP3_NORMAL_MODE)
  {
    if (OutputDataRate > BMP3_ODR_0_001_HZ)
    {
      printf("error data rate \n");
      return false;
    }
    the_sensor.settings.odr_filter.odr = OutputDataRate;
    settings_sel |= BMP3_ODR_SEL;
  }

  /* set interrupt settings */
  //TODO check if interrupt settings are ok
  the_sensor.settings.int_settings.drdy_en = 
      (PowerMode == BMP3_NORMAL_MODE && DataReadyInterrupt) ? BMP3_ENABLE : BMP3_DISABLE;
  //the_sensor.settings.int_settings.latch = BMP3_INT_PIN_NON_LATCH; //BMP3_INT_PIN_LATCH
  //the_sensor.settings.int_settings.level = BMP3_INT_PIN_ACTIVE_HIGH; //BMP3_INT_PIN_ACTIVE_LOW
  //the_sensor.settings.int_settings.output_mode = BMP3_INT_PIN_PUSH_PULL; //BMP3_INT_PIN_OPEN_DRAIN
  settings_sel |= BMP3_DRDY_EN_SEL; // | BMP3_LEVEL_SEL | BMP3_LATCH_SEL | BMP3_OUTPUT_MODE_SEL;

  /* set sensor settings */
  rslt = bmp3_set_sensor_settings(settings_sel, &the_sensor);
  if (rslt != BMP3_OK)
  {
    printf("error set sensor settings \n");
    return false;
  }

  #ifdef BMP3XX_DEBUG
    bmp3_get_sensor_settings(&the_sensor);
    Serial.printf("temp en: %u, temp os: %u, press en: %u, press os: %u, odr: %u", the_sensor.settings.temp_en, the_sensor.settings.odr_filter.temp_os, the_sensor.settings.press_en, the_sensor.settings.odr_filter.press_os, the_sensor.settings.odr_filter.odr);
  #endif

  /* set power mode */
  if (PowerMode == BMP3_NORMAL_MODE || PowerMode == BMP3_FORCED_MODE || PowerMode == BMP3_SLEEP_MODE)
  {
    _forcedModeEnabled = PowerMode == BMP3_FORCED_MODE;
    the_sensor.settings.op_mode = PowerMode;
   
    rslt = bmp3_set_op_mode(&the_sensor);
    if (rslt != BMP3_OK)
    {
      printf("error set power mode \n");
      if (rslt == BMP3_E_INVALID_ODR_OSR_SETTINGS)
        printf("BMP3_E_INVALID_ODR_OSR_SETTINGS \n");
      return false;
    }
  }
  else
    return false;

  return true;
}

/**************************************************************************/
/*!
    @brief Calculates the Sea level pressure (in Pa).

    @param  atmospheriquePressure      local pressure in Pa
    @param  yourActualAltitude         altitude in meters at your actual location
    @return Sea level pressure in Pa
*/
/**************************************************************************/
float Adafruit_BMP3XX::getSeaLevelPressure(double atmospheriquePressure, double yourActualAltitude)
{
  return (atmospheriquePressure / pow(1.0f - (yourActualAltitude / 44330.0f), 5.255f));
}

/**************************************************************************/
/*!
    @brief Calculate altitude from Pressure & Sea level pressure

    @param  [double] atmospheriquePressure      local pressure in Pa
    @param  [double] seaLevelPressure           sea level pressure in Pa
    @return [double] altitude in meter
*/
/**************************************************************************/
double Adafruit_BMP3XX::getAltitude(double pressure, double seaLevelPressure)
{
    return (44330.0f * (1.0f - pow((double)pressure / (double)seaLevelPressure, 0.1902949f)));
}
