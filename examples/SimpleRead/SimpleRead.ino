/******************************************************************************
SimpleRead.ino

Spresense Users
March 18 , 2023

Description:
Example using up to one LIS2DW12s via I2C channel.

Resources:
Uses Wire.h for i2c operation

Distributed as-is; no warranty is given.
******************************************************************************/

#include <LIS2DW12.h>
#include "Wire.h"

SensorSettings LIS2DW12_settings;

//Create two instances of the driver class
LIS2DW12 Sensor( I2C_MODE, 0x19 );


void set_sensor() {
  LIS2DW12_settings.mode       = 0;    // 0 = low power, 1 = high performance, 2 = single data conversion
  LIS2DW12_settings.lpMode     = 1;    // 1 = lp mode 1 (12 bit), 2 = lp mode 2 (14 bit) ...
  LIS2DW12_settings.odr        = 200;  // Hz. Default is 0 = power down
  
  // CTRL2
  LIS2DW12_settings.csPuDisc   = 0;    // 0 = pull-up connected to CS pin
  LIS2DW12_settings.i2cDisable = 0;    // 0 = i2c enable, 1 = i2c disable
  
  // CTRL3
  LIS2DW12_settings.ppOd       = 0;    // 0 = push-pull, 1 = open-drain
  LIS2DW12_settings.lir        = 1;    // 0 = interrupt not latched, 1 = interrupt signal latched
  LIS2DW12_settings.hiActive   = 1;    // 0 = active high, 1 = active low
  
  // CTRL6
  LIS2DW12_settings.fs         = 2;    // 2g, 4g, 8g, 16g
  LIS2DW12_settings.lowNoise   = 1;    // 1 = low noise enabled
  
  LIS2DW12_settings.tapTh      = 0x0C;   // threshold for tap detection
  LIS2DW12_settings.latency    = 0x30;   // latency for double tap detection ((0x40 >> 4) * 32 / ODR)
  LIS2DW12_settings.quiet      = 0x08;   // quiet time window for double tap detection ((0x08 >> 2) * 4 / ODR)
  LIS2DW12_settings.shock      = 0x02;   // shock time window for double tap detection (0x02 * 8 / ODR)
  
  LIS2DW12_settings.accelSensitivity = 0.244;  // set correct sensitivity from LIS2DW12 Application Notes (FS = 2g, resolution = 14bit)
                                               // this is a function of full scale setting (FS) and resolution (12 or 14 bit format)
}

void setup() {

  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000); //relax...
  Serial.println("Processor came out of reset.\n");

  //Call .begin() to configure the IMUs
  set_sensor();

  if( Sensor.begin(LIS2DW12_settings) != 0 )
  {
	  Serial.println("Problem starting the sensor at 0x19.");
  }
  else
  {
	  Serial.println("Sensor at 0x19 started.");
  }
}

void loop()
{
  //Get all parameters
  Serial.print(" X = ");
  Serial.println(Sensor.readFloatAccelX(), 4);
  Serial.print(" Y = ");
  Serial.println(Sensor.readFloatAccelY(), 4);
  Serial.print(" Z = ");
  Serial.println(Sensor.readFloatAccelZ(), 4);

  Serial.print("\nThermometer:\n");
  Serial.print(" Degrees C = ");
  Serial.println(Sensor.readTempCLowRes(), 4);
  Serial.print(" Degrees F = ");
  Serial.println(Sensor.readTempFLowRes(), 4);
  
  Serial.print("Sensor Bus Errors Reported:\n");
  Serial.print(" All '1's = ");
  Serial.println(Sensor.allOnesCounter);
  Serial.print(" Non-success = ");
  Serial.println(Sensor.nonSuccessCounter);
  Serial.println("\n\n");

  sleep(1);
}
