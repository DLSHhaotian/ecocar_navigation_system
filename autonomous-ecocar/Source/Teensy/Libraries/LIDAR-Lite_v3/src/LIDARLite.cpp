/*------------------------------------------------------------------------------

  LIDARLite Arduino Library
  LIDARLite.cpp

  This library provides quick access to the basic functions of LIDAR-Lite
  via the Arduino interface. Additionally, it can provide a user of any
  platform with a template for their own application code.

  Copyright (c) 2016 Garmin Ltd. or its subsidiaries.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

  http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

------------------------------------------------------------------------------*/

#include <Arduino.h>
#include <i2c_t3.h>
#include <stdarg.h>
#include "LIDARLite.h"

/*------------------------------------------------------------------------------
  Constructor

  Use LIDARLite::begin to initialize.
------------------------------------------------------------------------------*/
LIDARLite::LIDARLite() {}

/*------------------------------------------------------------------------------
  Begin

  Starts the sensor and I2C.

  Parameters
  ------------------------------------------------------------------------------
  configuration: Default 0. Selects one of several preset configurations.
  fasti2c: Default 100 kHz. I2C base frequency.
	If true I2C frequency is set to 400kHz.
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
	operating manual for instructions.
------------------------------------------------------------------------------*/
bool LIDARLite::begin(int configuration, bool fasti2c, char lidarliteAddress, int wireNum) {
	_wireNum = wireNum;
	if(_wireNum == 0) {
		Wire.begin(); // Start I2C
		if(fasti2c) {
			Wire.setClock(400000UL); // Set I2C frequency to 400kHz, for Arduino Due
		}
	} else if(_wireNum == 1) {
		Wire1.begin();
		if(fasti2c) {
			Wire1.setClock(400000UL); // Set I2C frequency to 400kHz, for Arduino Due
		}
	}
	Wire.setDefaultTimeout(1000);
	return configure(configuration, lidarliteAddress); // Configuration settings
} /* LIDARLite::begin */

/*------------------------------------------------------------------------------
  Configure

  Selects one of several preset configurations.

  Parameters
  ------------------------------------------------------------------------------
  configuration:  Default 0.
	0: Default mode, balanced performance.
	1: Short range, high speed. Uses 0x1d maximum acquisition count.
	2: Default range, higher speed short range. Turns on quick termination
		detection for faster measurements at short range (with decreased
		accuracy)
	3: Maximum range. Uses 0xff maximum acquisition count.
	4: High sensitivity detection. Overrides default valid measurement detection
		algorithm, and uses a threshold value for high sensitivity and noise.
	5: Low sensitivity detection. Overrides default valid measurement detection
		algorithm, and uses a threshold value for low sensitivity and noise.
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
	operating manual for instructions.
------------------------------------------------------------------------------*/
bool LIDARLite::configure(int configuration, char lidarliteAddress) {
	if(_wireNum == 0) {
		Wire.beginTransmission(lidarliteAddress);
		byte error = Wire.endTransmission();
		if(error != 0) return false;
	} else if(_wireNum == 1) {
		Wire1.beginTransmission(lidarliteAddress);
		byte error = Wire1.endTransmission();
		if(error != 0) return false;
	}
	switch(configuration) {
	case 0: // Default mode, balanced performance
		write(0x02, 0x80, lidarliteAddress); // Default
		write(0x04, 0x08, lidarliteAddress); // Default
		write(0x1c, 0x00, lidarliteAddress); // Default
		break;

	case 1: // Short range, high speed
		write(0x02, 0x1d, lidarliteAddress);
		write(0x04, 0x08, lidarliteAddress); // Default
		write(0x1c, 0x00, lidarliteAddress); // Default
		break;

	case 2: // Default range, higher speed short range
		write(0x02, 0x80, lidarliteAddress); // Default
		write(0x04, 0x00, lidarliteAddress);
		write(0x1c, 0x00, lidarliteAddress); // Default
		break;

	case 3: // Maximum range
		write(0x02, 0xff, lidarliteAddress);
		write(0x04, 0x08, lidarliteAddress); // Default
		write(0x1c, 0x00, lidarliteAddress); // Default
		break;

	case 4: // High sensitivity detection, high erroneous measurements
		write(0x02, 0x80, lidarliteAddress); // Default
		write(0x04, 0x08, lidarliteAddress); // Default
		write(0x1c, 0x80, lidarliteAddress);
		break;

	case 5: // Low sensitivity detection, low erroneous measurements
		write(0x02, 0x80, lidarliteAddress); // Default
		write(0x04, 0x08, lidarliteAddress); // Default
		write(0x1c, 0xb0, lidarliteAddress);
		break;
	}
	return true;
} /* LIDARLite::configure */

/*------------------------------------------------------------------------------
  Reset

  Reset device. The device reloads default register settings, including the
  default I2C address. Re-initialization takes approximately 22ms.

  Parameters
  ------------------------------------------------------------------------------
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
	operating manual for instructions.
------------------------------------------------------------------------------*/
void LIDARLite::reset(char lidarliteAddress) {
	write(0x00, 0x00, lidarliteAddress);
} /* LIDARLite::reset */

/*------------------------------------------------------------------------------
  Distance

  Take a distance measurement and read the result.

  Process
  ------------------------------------------------------------------------------
  1.  Write 0x04 or 0x03 to register 0x00 to initiate an aquisition.
  2.  Read register 0x01 (this is handled in the read() command)
	  - if the first bit is "1" then the sensor is busy, loop until the first
		bit is "0"
	  - if the first bit is "0" then the sensor is ready
  3.  Read two bytes from register 0x8f and save
  4.  Shift the first value from 0x8f << 8 and add to second value from 0x8f.
	  The result is the measured distance in centimeters.

  Parameters
  ------------------------------------------------------------------------------
  biasCorrection: Default true. Take aquisition with receiver bias
	correction. If set to false measurements will be faster. Receiver bias
	correction must be performed periodically. (e.g. 1 out of every 100
	readings).
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
	operating manual for instructions.
------------------------------------------------------------------------------*/
int LIDARLite::distance(bool biasCorrection, char lidarliteAddress) {
	if(biasCorrection) {
		// Take acquisition & correlation processing with receiver bias correction
		write(0x00, 0x04, lidarliteAddress);
	} else {
		// Take acquisition & correlation processing without receiver bias correction
		write(0x00, 0x03, lidarliteAddress);
	}
	// Array to store high and low bytes of distance
	byte distanceArray[2];
	// Read two bytes from register 0x8f (autoincrement for reading 0x0f and 0x10)
	bool success = read(0x8f, 2, distanceArray, true, lidarliteAddress);
	if(!success) return -1;
	// Shift high byte and add to low byte
	int distance = (distanceArray[0] << 8) + distanceArray[1];
	return(distance);
} /* LIDARLite::distance */

/*------------------------------------------------------------------------------
  Write

  Perform I2C write to device.

  Parameters
  ------------------------------------------------------------------------------
  myAddress: register address to write to.
  myValue: value to write.
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
	operating manual for instructions.
------------------------------------------------------------------------------*/
bool LIDARLite::write(char myAddress, char myValue, char lidarliteAddress) {
	if(_wireNum == 0) {
		Wire.beginTransmission((int)lidarliteAddress);
		Wire.write((int)myAddress); // Set register for write
		Wire.write((int)myValue); // Write myValue to register
		// A nack means the device is not responding, report the error over serial
		int nackCatcher = Wire.endTransmission();
		if(nackCatcher != 0) return false;
	} else if(_wireNum == 1) {
		Wire1.beginTransmission((int)lidarliteAddress);
		Wire1.write((int)myAddress); // Set register for write
		Wire1.write((int)myValue); // Write myValue to register
		// A nack means the device is not responding, report the error over serial
		int nackCatcher = Wire1.endTransmission();
		if(nackCatcher != 0) return false;
	}

	delay(1); // 1 ms delay for robustness with successive reads and writes
	
	return true;
} /* LIDARLite::write */

/*------------------------------------------------------------------------------
  Read

  Perform I2C read from device. Will detect an unresponsive device and report
  the error over serial. The optional busy flag monitoring
  can be used to read registers that are updated at the end of a distance
  measurement to obtain the new data.

  Parameters
  ------------------------------------------------------------------------------
  myAddress: register address to read from.
  numOfBytes: numbers of bytes to read. Can be 1 or 2.
  arrayToSave: an array to store the read values.
  monitorBusyFlag: if true, the routine will repeatedly read the status
	register until the busy flag (LSB) is 0.
------------------------------------------------------------------------------*/
bool LIDARLite::read(char myAddress, int numOfBytes, byte arrayToSave[2], bool monitorBusyFlag, char lidarliteAddress) {
	bool result = true;
	int busyFlag = 0; // busyFlag monitors when the device is done with a measurement
	if(monitorBusyFlag) {
		busyFlag = 1; // Begin read immediately if not monitoring busy flag
	}
	int busyCounter = 0; // busyCounter counts number of times busy flag is checked, for timeout

	while(busyFlag != 0) // Loop until device is not busy
	{
		if(_wireNum == 0) {
			// Read status register to check busy flag
			Wire.beginTransmission((int)lidarliteAddress);
			Wire.write(0x01); // Set the status register to be read
			// A nack means the device is not responding, report the error over serial
			int nackCatcher = Wire.endTransmission();
			if(nackCatcher != 0) result = false;
			Wire.requestFrom((int)lidarliteAddress, 1); // Read register 0x01
			busyFlag = bitRead(Wire.read(), 0); // Assign the LSB of the status register to busyFlag
		} else if(_wireNum == 1) {
			// Read status register to check busy flag
			Wire1.beginTransmission((int)lidarliteAddress);
			Wire1.write(0x01); // Set the status register to be read
			// A nack means the device is not responding, report the error over serial
			int nackCatcher = Wire1.endTransmission();
			if(nackCatcher != 0) result = false;
			Wire1.requestFrom((int)lidarliteAddress, 1); // Read register 0x01
			busyFlag = bitRead(Wire1.read(), 0); // Assign the LSB of the status register to busyFlag
		}
		busyCounter++; // Increment busyCounter for timeout

		// Handle timeout condition, exit while loop and goto bailout
		if(busyCounter > 9999) {
			goto bailout;
		}
	}

	// Device is not busy, begin read
	if(busyFlag == 0) {
		if(_wireNum == 0) {
			Wire.beginTransmission((int)lidarliteAddress);
			Wire.write((int)myAddress); // Set the register to be read
			// A nack means the device is not responding, report the error over serial
			int nackCatcher = Wire.endTransmission();
			if(nackCatcher != 0) result = false;
			// Perform read of 1 or 2 bytes, save in arrayToSave
			Wire.requestFrom((int)lidarliteAddress, numOfBytes);
			int i = 0;
			if(numOfBytes <= Wire.available()) {
				while(i < numOfBytes) {
					arrayToSave[i] = Wire.read();
					i++;
				}
			}
		} else if(_wireNum == 1) {
			Wire1.beginTransmission((int)lidarliteAddress);
			Wire1.write((int)myAddress); // Set the register to be read
										// A nack means the device is not responding, report the error over serial
			int nackCatcher = Wire1.endTransmission();
			if(nackCatcher != 0) result = false;
			// Perform read of 1 or 2 bytes, save in arrayToSave
			Wire1.requestFrom((int)lidarliteAddress, numOfBytes);
			int i = 0;
			if(numOfBytes <= Wire1.available()) {
				while(i < numOfBytes) {
					arrayToSave[i] = Wire1.read();
					i++;
				}
			}
		}
	}
	// bailout reports error over serial
	if(busyCounter > 9999) {
	bailout:
		busyCounter = 0;
		result = false;
	}
	return result;
} /* LIDARLite::read */

/*------------------------------------------------------------------------------
  Correlation Record To Serial

  The correlation record used to calculate distance can be read from the device.
  It has a bipolar wave shape, transitioning from a positive going portion to a
  roughly symmetrical negative going pulse. The point where the signal crosses
  zero represents the effective delay for the reference and return signals.

  Process
  ------------------------------------------------------------------------------
  1.  Take a distance reading (there is no correlation record without at least
	  one distance reading being taken)
  2.  Select memory bank by writing 0xc0 to register 0x5d
  3.  Set test mode select by writing 0x07 to register 0x40
  4.  For as many readings as you want to take (max is 1024)
	  1.  Read two bytes from 0xd2
	  2.  The Low byte is the value from the record
	  3.  The high byte is the sign from the record

  Parameters
  ------------------------------------------------------------------------------
  separator: the separator between serial data words
  numberOfReadings: Default: 256. Maximum of 1024
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
	operating manual for instructions.
------------------------------------------------------------------------------*/
/*void LIDARLite::correlationRecordToSerial(char separator, int numberOfReadings, char lidarliteAddress) {

	// Array to store read values
	byte correlationArray[2];
	// Var to store value of correlation record
	int correlationValue = 0;
	//  Selects memory bank
	write(0x5d, 0xc0, lidarliteAddress);
	// Test mode enable
	write(0x40, 0x07, lidarliteAddress);
	for(int i = 0; i < numberOfReadings; i++) {
		// Select single byte
		read(0xd2, 2, correlationArray, false, lidarliteAddress);
		//  Low byte is the value of the correlation record
		correlationValue = correlationArray[0];
		// if upper byte lsb is set, the value is negative
		if((int)correlationArray[1] == 1) {
			correlationValue |= 0xff00;
		}
		Serial.print((int)correlationValue);
		Serial.print(separator);
	}
	// test mode disable
	write(0x40, 0x00, lidarliteAddress);
} // LIDARLite::correlationRecordToSerial */

/* =============================================================================
  Change I2C Address for Single Sensor
  LIDAR-Lite now has the ability to change the I2C address of the sensor and
  continue to use the default address or disable it. This function only works
  for single sensors. When the sensor powers off and restarts this value will
  be lost and will need to be configured again.
  There are only certain address that will work with LIDAR-Lite so be sure to
  review the "Notes" section below
  Process
  ------------------------------------------------------------------------------
  1.  Read the two byte serial number from register 0x96
  2.  Write the low byte of the serial number to 0x18
  3.  Write the high byte of the serial number to 0x19
  4.  Write the new address you want to use to 0x1a
  5.  Choose wheather to user the default address or not (you must to one of the
	  following to commit the new address):
	  1.  If you want to keep the default address, write 0x00 to register 0x1e
	  2.  If you do not want to keep the default address write 0x08 to 0x1e
  Parameters
  ------------------------------------------------------------------------------
  - newI2cAddress: the hex value of the I2C address you want the sensor to have
  - disablePrimaryAddress (optional): true/false value to disable the primary
	address, default is false (i.e. leave primary active)
  - currentLidarLiteAddress (optional): the default is 0x62, but can also be any
	value you have previously set (ex. if you set the address to 0x66 and dis-
	abled the default address then needed to change it, you would use 0x66 here)
  Example Usage
  ------------------------------------------------------------------------------
  1.  //  Set the value to 0x66 with primary address active and starting with
	  //  0x62 as the current address
	  myLidarLiteInstance.changeAddress(0x66);
  Notes
  ------------------------------------------------------------------------------
	Possible Address for LIDAR-Lite
	7-bit address in binary form need to end in "0". Example: 0x62 = 01100010 so
	that works well for us. Essentially any even numbered hex value will work
	for 7-bit address.
	8-bit read address in binary form need to end in "00". Example: the default
	8-bit read address for LIDAR-Lite is 0xc4 = 011000100. Essentially any hex
	value evenly divisable by "4" will work.
  =========================================================================== */
/*unsigned char LIDARLite::changeAddress(char newI2cAddress, bool disablePrimaryAddress, char currentLidarLiteAddress) {
	//  Array to save the serial number
	unsigned char serialNumber[2];
	unsigned char newI2cAddressArray[1];

	//  Read two bytes from 0x96 to get the serial number
	read(0x96, 2, serialNumber, false, currentLidarLiteAddress);
	//  Write the low byte of the serial number to 0x18
	write(0x18, serialNumber[0], currentLidarLiteAddress);
	//  Write the high byte of the serial number of 0x19
	write(0x19, serialNumber[1], currentLidarLiteAddress);
	//  Write the new address to 0x1a
	write(0x1a, newI2cAddress, currentLidarLiteAddress);


	while(newI2cAddress != newI2cAddressArray[0]) {
		read(0x1a, 1, newI2cAddressArray, false, currentLidarLiteAddress);
	}
	Serial.println("Address successfully changed!");
	//  Choose whether or not to use the default address of 0x62
	if(disablePrimaryAddress) {
		write(0x1e, 0x08, currentLidarLiteAddress);
	} else {
		write(0x1e, 0x00, currentLidarLiteAddress);
	}

	return newI2cAddress;
} // */

/* =============================================================================
  Change I2C Address for Multiple Sensors
  Using the new I2C address change feature, you can also change the address for
  multiple sensors using the PWR_EN line connected to Arduino's digital pins.
  Address changes will be lost on power off.
  Process
  ------------------------------------------------------------------------------
  1.
  Parameters
  ------------------------------------------------------------------------------
  - numberOfSensors: int representing the number of sensors you have connected
  - pinArray: array of the digital pins your sensors' PWR_EN line is connected
	to
  - i2cAddressArray: array of the I2C address you want to assign to your sen-
	sors, the order should reflect the order of the pinArray (see not for poss-
	ible addresses below)
  - usePartyLine(optional): true/false value of whether or not to leave 0x62
	available to all sensors for write (default is false)
  Example Usage
  ------------------------------------------------------------------------------
  1.  //  Assign new address to the sensors connected to sensorsPins and disable
	  //  0x62 as a partyline to talk to all of the sensors
	  int sensorPins[] = {2,3,4};
	  unsigned char addresses[] = {0x66,0x68,0x64};
	  myLidarLiteInstance.changeAddressMultisensor(3,sensorPins,addresses);
  Notes
  ------------------------------------------------------------------------------
	Possible Address for LIDAR-Lite
	7-bit address in binary form need to end in "0". Example: 0x62 = 01100010 so
	that works well for us. Essentially any even numbered hex value will work
	for 7-bit address.
	8-bit read address in binary form need to end in "00". Example: the default
	8-bit read address for LIDAR-Lite is 0xc4 = 011000100. Essentially any hex
	value evenly divisable by "4" will work.
  =========================================================================== */
/*void LIDARLite::changeAddressMultiPwrEn(int numOfSensors, int *pinArray, unsigned char *i2cAddressArray, bool usePartyLine) {
	for(int i = 0; i < numOfSensors; i++) {
		pinMode(pinArray[i], OUTPUT); // Pin to first LIDAR-Lite Power Enable line
		delay(2);
		digitalWrite(pinArray[i], HIGH);
		delay(20);
		configure(1);
		changeAddress(i2cAddressArray[i], true, 0x62); // We have to turn off the party line to actually get these to load
	}
	if(usePartyLine) {
		for(int i = 0; i < numOfSensors; i++) {
			write(0x1e, 0x00, i2cAddressArray[i]);
		}
	}
} // */