/******************************************************************************
InterruptHWTapConfig.ino

Ion Bold
October 18 , 2018

Description:
Example using the LIS2DW12 interrupts.

This sketch demonstrates one way to detect single and double-tap events using
hardware interrupt pins. The LIS2DW12 pulses the int1 line once after the first
tap, then again if a second tap comes in.

The configuration is determined by reading the LIS2DW12 datasheet and application
note, then driving hex values to the registers of interest to set the appropriate
bits.

Resources:
Uses Wire.h for i2c operation
Uses SPI.h for SPI operation

Development environment specifics:
Arduino IDE 1.8.5

Hardware connections:
Connect I2C SDA line to A4
Connect I2C SCL line to A5
Connect GND and 3.3v power to the IMU
Connect INT1 to pin 2 -- Note:  the atmega has 5v input and the LIS2DW12 is 3.3v
output.  This is OK because the input is high impedance and 3.3v is a logic '1'
for 5v processors.  The signal is correctly detected and nothing is damaged.

Do not configure pin 2 as OUTPUT!

Distributed as-is; no warranty is given.
******************************************************************************/

#include <LIS2DW12.h>
#include "Wire.h"

SensorSettings LIS2DW12_settings;

//Interrupt variables
#define int1Pin 22 
uint8_t int1Status = 0;
uint8_t result;

//Create an instances of the driver class
LIS2DW12 Sensor( I2C_MODE, 0x19 );

void set_sensor()
{
  LIS2DW12_settings.mode       = 1;    // 0 = low power, 1 = high performance, 2 = single data conversion
  LIS2DW12_settings.lpMode     = 0;    // 1 = lp mode 1 (12 bit), 2 = lp mode 2 (14 bit) ...
  LIS2DW12_settings.odr        = 200;  // Hz. Default is 0 = power down
  
  // CTRL2
  LIS2DW12_settings.csPuDisc   = 0;    // 0 = pull-up connected to CS pin
  LIS2DW12_settings.i2cDisable = 0;    // 0 = i2c enable, 1 = i2c disable
  
  // CTRL3
  LIS2DW12_settings.ppOd       = 0;    // 0 = push-pull, 1 = open-drain
  LIS2DW12_settings.lir        = 0;    // 0 = interrupt not latched, 1 = interrupt signal latched
  LIS2DW12_settings.hiActive   = 0;    // 0 = active high, 1 = active low
  
  // CTRL6
  LIS2DW12_settings.fs         = 4;    // 2g, 4g, 8g, 16g
  LIS2DW12_settings.lowNoise   = 1;    // 1 = low noise enabled
  
  LIS2DW12_settings.tapTh      = 0x06;   // threshold for tap detection
  LIS2DW12_settings.latency    = 0x20;   // latency for double tap detection ((0x40 >> 4) * 32 / ODR)
  LIS2DW12_settings.quiet      = 0x02;   // quiet time window for double tap detection ((0x08 >> 2) * 4 / ODR)
  LIS2DW12_settings.shock      = 0x02;   // shock time window for double tap detection (0x02 * 8 / ODR)
  
  LIS2DW12_settings.accelSensitivity = 0.488;  // set correct sensitivity from LIS2DW12 Application Notes (FS = 2g, resolution = 14bit)
                                               // this is a function of full scale setting (FS) and resolution (12 or 14 bit format)
}

void setup()
{
	Serial.begin(115200);
	delay(1000); //relax...
	Serial.println("Processor came out of reset.\n");

	//Call .beginCore() to configure the IMU
	set_sensor();
	if( Sensor.begin(LIS2DW12_settings) != 0 )
	{
		Serial.print("Error at beginCore().\n");
	}
	else
	{
		Serial.print("\nbeginCore() passed.\n");
	}

	if( Sensor.initDoubleTap(3) )            // 0: only X axis, 1: only Y axis, 2: only Z axis, >2: all axis
	{
		Serial.println("Problem configuring the device.");
	}
	else
	{
		Serial.println("Device O.K.");
	}	

	//Configure the atmega interrupt pin
	pinMode(int1Pin, INPUT);
	attachInterrupt(int1Pin, int1ISR, RISING);

}


void loop()
{
	if(int1Status)  //If ISR has been serviced at least once
	{
		Serial.print("Double-tap event\n");
		Sensor.readRegister(&result, 0x3B);
		//Clear the ISR counter
		int1Status = 0;
	}
	usleep(10*1000);
}

void int1ISR()
{
	puts("Interrupt serviced.");
	int1Status = 1;
}
