// INTERFACE.h

#ifndef _INTERFACE_h
#define _INTERFACE_h

#include "Arduino.h"

#define ENCODER_USE_INTERRUPTS

#include <Encoder.h>
//#include <i2c_t3.h>	// TODO For Sonar
#include "QuadDecodeSimple.h"

#define ANALOG_READ_RESOLUTION 16
#define ANALOG_READ_MAX 65535.0f	// 2^ANALOG_READ_RESOLUTION - 1
#define BATTERY_VOLTAGE_DIVIDER_FACTOR 50.0f / 11.0f * 3.3f
#define STEERING_ENCODER_A 29		// FTM2 A (Brown)
#define STEERING_ENCODER_B 30		// FTM2 B (White)
#define STEERING_POTENTIOMETER 17	// ADC A3
#define BRAKE_POTENTIOMETER 34		// ADC 8 = ADC A15 (Brown)
#define EMERGENCY_BUTTON 31			// ADC 9 = ADC A12 (White)
#define BATTERY_VOLTAGE 32			// ADC A13
#define BUZZER_PIN 35				// Buzzer
#define AUTONOMOUS_STATE_PIN 18		// Pin for autonomous state (HIGH = AUTOPILOT_ACTIVE)
#define EMERGENCY_BUTTON_THRESHOLD  30000	// Values below this will trigger emergency button pushed
#define DISTANCE_FRONT_SENSOR_I2C_ADDRESS 240	// I2C Address of the front distance sensor (ultrasonic sonar)
#define DISTANCE_FRONT_READ_PERIOD 150	// [ms]

// Steering Encoder
QuadDecode_t QuadDecode;

// Front distance timing
uint32_t lastDistanceFrontRequestTime = 0;
float distanceFront;

void inline interfaceInit() {
	analogReadResolution(ANALOG_READ_RESOLUTION);
	pinMode(STEERING_POTENTIOMETER, INPUT);
	pinMode(BRAKE_POTENTIOMETER, INPUT);
	pinMode(BATTERY_VOLTAGE, INPUT);
	pinMode(EMERGENCY_BUTTON, INPUT);
	//pinMode(AUTONOMOUS_STATE_PIN, OUTPUT);	// TODO
}

// Set autonomous state
void inline autoState(bool on) {
	if(on) digitalWrite(AUTONOMOUS_STATE_PIN, HIGH);
	else digitalWrite(AUTONOMOUS_STATE_PIN, LOW);
}

// Reset the steering encoder
void inline steeringEncoderReset() {
	QuadDecode.setCounter2(0);
}

// Steering encoder
int32_t inline steeringEncoderRead() {
	return QuadDecode.getCounter2();
}

// Steering potentiometer
int inline steeringPotentiometerRead() {
	return analogRead(STEERING_POTENTIOMETER);
}

// Brake potentiometer
int inline brakePotentiometerRead() {
	return analogRead(BRAKE_POTENTIOMETER);
}

// Emergency button (are actuators powered?): Returns false when emergency button has been pushed
bool inline emergencyButtonRead() {
	int emergencyVoltage = analogRead(EMERGENCY_BUTTON);
	if(emergencyVoltage > EMERGENCY_BUTTON_THRESHOLD) return true;
	return false;
}

// Read the battery voltage (from the supply to the Auto board)
float inline batteryVoltageRead() {
	return (analogRead(BATTERY_VOLTAGE) * BATTERY_VOLTAGE_DIVIDER_FACTOR / ANALOG_READ_MAX);
}

boolean inline frontDistanceRequest() {
	/*byte bit8address = DISTANCE_FRONT_SENSOR_I2C_ADDRESS;
	bit8address = bit8address & B11111110;
	Wire1.beginTransmission(bit8address);
	Wire1.write(81);
	byte result = Wire1.endTransmission();
	return (result == 0); */		// TODO For Sonar
	return false;
}

// Start the front distance sensor (ultrasonic sonar)
// Returns: true if sensor is connected
boolean inline frontDistanceInit() {
	//Wire1.begin(I2C_MASTER, 0x00, I2C_PINS_37_38, I2C_PULLUP_EXT, 100000);		// TODO For Sonar
	return frontDistanceRequest();
}

// Read from the front distance sensor (ultrasonic sonar)
float inline distanceFrontRead() {
	/*if(lastDistanceFrontRequestTime == 0) {
		boolean result = frontDistanceRequest();
		if(result) {
			// Range reading request sent successfully
			// In about 100ms (DISTANCE_FRONT_READ_PERIOD) the reading will be ready
			lastDistanceFrontRequestTime = millis();
		}
	} else if(millis() - lastDistanceFrontRequestTime > DISTANCE_FRONT_READ_PERIOD) {
		int range = -1;
		byte rangeHighByte = 0;
		byte rangeLowByte = 0;
		Wire1.requestFrom(DISTANCE_FRONT_SENSOR_I2C_ADDRESS, 2);
		if(Wire1.available() >= 2) {
			rangeHighByte = Wire1.read();
			rangeLowByte = Wire1.read();
			range = (rangeHighByte * 256) + rangeLowByte;
		}
		// The sensor returns the range in cm, so divide by 100 to get m
		if(range != -1) {
			lastDistanceFrontRequestTime = 0;
			return range / 100.0f;
		}
		return distanceFront;
	}*/ 	// TODO For Sonar
	return distanceFront;
}
#endif

