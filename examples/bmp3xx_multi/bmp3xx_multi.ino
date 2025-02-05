#include "Adafruit_BMP3XX.h"

Adafruit_BMP3XX bmp1;
Adafruit_BMP3XX bmp2;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Adafruit BMP388 / BMP390 test");

  if (! bmp1.begin_I2C()) {  
    Serial.println("Could not find a valid BMP sensor #1, check wiring!");
    while (1);
  }

  if (! bmp2.begin_I2C(0x76)) {  
    Serial.println("Could not find a valid BMP sensor #2, check wiring!");
    while (1);
  }

  Serial.println("Initialization Success!");

  Serial.print("Configuring BMP #1...");
  bmp1.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp1.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp1.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp1.setOutputDataRate(BMP3_ODR_50_HZ);
  Serial.println("DONE.");

  Serial.print("Configuring BMP #2...");
  bmp2.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp2.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp2.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp2.setOutputDataRate(BMP3_ODR_50_HZ);
  Serial.println("DONE.");

  Serial.println("----[START READING]----------------------------");
}

void loop() {
  if (! bmp1.performReading()) {
    Serial.println("Failed to perform reading BMP #1 :(");
    return;
  }
  Serial.print("BMP #1 Temperature = ");
  Serial.print(bmp1.temperature);
  Serial.print(" *C");
  Serial.print("     Pressure = ");
  Serial.print(bmp1.pressure / 100.0);
  Serial.println(" hPa");  

  if (! bmp2.performReading()) {
    Serial.println("Failed to perform reading BMP #2 :(");
    return;
  }
  Serial.print("BMP #2 Temperature = ");
  Serial.print(bmp2.temperature);
  Serial.print(" *C");
  Serial.print("     Pressure = ");
  Serial.print(bmp2.pressure / 100.0);
  Serial.println(" hPa"); 

  delay(2000);
}
