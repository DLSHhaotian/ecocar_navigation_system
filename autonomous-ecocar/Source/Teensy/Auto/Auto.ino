// Auto 2018 v2.0
// Autonomous Ecocar: Embedded Software for hardware interface
// Technical University of Denmark
// Henning Si Hoj

//#define DEBUG

#include <ArduinoJson.h>
#include <LIDARLite.h>
#include <canbus.h>
#include <tunes.h>

#include "interface.h"
#include "encoder.h"
#include "led.h"

#define SEND_BUFFER_SIZE 400
#define READ_BUFFER_SIZE 400
#define WATCHDOG_PERIOD 1000	// [us]
#define SERIAL_TIMEOUT_MS 1000	// [ms]
#define CAN_SEND_PERIOD 50		// [ms]
#define I2C_PERIOD 100			// [ms]

// These uint32 for millis are sufficient for 49 days
static uint32_t canTiming = 0;
static uint32_t i2cTiming = 0;
static uint32_t lastSerialReceivedTime = 0;
static uint32_t lastWatchdogActionTime = 0;

// Internal variables
IntervalTimer watchdogTimer;
static bool isFrontDistanceSensorConnected = false;
static bool isLeftLidarConnected = false;
static bool isRightLidarConnected = false;
static LIDARLite leftLidarLite;
static LIDARLite rightLidarLite;
static int distanceLeft;
static int distanceRight;

// Locally obtained values to send on USB
static float brakePotentiometer;
static int32_t steeringEncoder;
static float steeringPotentiometer;
static float batteryVoltage;
static bool emergencyButton;

// Initialization for values to read from USB and send on CANBUS
static uint8_t burn = 0;
static uint8_t light = 0;
static uint8_t blinkLeft = 0;
static uint8_t blinkRight = 0;
static uint8_t horn = 0;
static uint8_t parking = 0;
static uint8_t autopilotActive = 0;

// Initialization for values to read from CANBUS and send on USB
static float speedWheel;
static float brakePressure1;
static float brakePressure2;
static float gear;
static float distanceWheel;
static uint8_t autopilot;
static float engineBatteryVoltage;

void setup() {
	// Startup delay
	delay(500);
	// Initialize serial over USB to PC
	Serial.begin(1000000);
	// Initialize pins
	interfaceInit();
	// Initialize LEDs
	LED_init();
	// Initialize CAN
	can_init(AUTO_ID);
	// Initialize Buzzer
	tunes_init(BUZZER_PIN);
	// Initialize front distance sensor (ultrasonic sonar)
	isFrontDistanceSensorConnected = frontDistanceInit();
	// Initialize LiDARs
	isLeftLidarConnected = leftLidarLite.begin(0, true, 0x62, 0);		// Wire 0
	delay(5);
	isRightLidarConnected = rightLidarLite.begin(0, true, 0x62, 1);		// Wire 1
	delay(5);
	// Set up watchdog
	watchdogTimer.begin(watchdogCallback, WATCHDOG_PERIOD);
	sei();
	delay(100);
	// Startup complete
	LED2On();
}

inline bool read() {
	// Read data
	// Example: {"b":0,"i":1,"l":1,"r":0,"h":0,"p":0,"a":0}
	StaticJsonBuffer<READ_BUFFER_SIZE> readBuffer;
	JsonObject& readValue = readBuffer.parse(Serial);
	if(readValue.success()) {
		// Extract data from JSON
		burn = readValue["b"];
		light = readValue["i"];
		blinkLeft = readValue["l"];
		blinkRight = readValue["r"];
		horn = readValue["h"];
		parking = readValue["p"];
		autopilotActive = readValue["a"];
		// Ensure boolean values
		if(burn) burn = 1;
		else burn = 0;
		if(light) light = 1;
		else light = 0;
		if(blinkLeft) blinkLeft = 1;
		else blinkLeft = 0;
		if(blinkRight) blinkRight = 1;
		else blinkRight = 0;
		if(horn) horn = 1;
		else horn = 0;
		if(parking) parking = 1;
		else parking = 0;
		if(autopilotActive) autopilotActive = 1;
		else autopilotActive = 0;
		// Set last received time
		lastSerialReceivedTime = millis();
		return true;
	} else {
		Serial.println("{\"error\":\"Invalid input\"}");
		// Clear receive buffer
		while(Serial.available()) {
			Serial.read();
		}
		// If no valid string was received, no data is sent back
		return false;
	}
}

inline void send() {
	// Send values
	StaticJsonBuffer<SEND_BUFFER_SIZE> sendBuffer;
	JsonObject& sendValue = sendBuffer.createObject();

	sendValue["sW"] = speedWheel;
	sendValue["p1"] = brakePressure1;
	sendValue["p2"] = brakePressure2;
	sendValue["bE"] = brakePotentiometer;
	sendValue["sE"] = steeringEncoder;
	sendValue["sP"] = steeringPotentiometer;
	sendValue["gr"] = gear;
	sendValue["dW"] = distanceWheel;
	sendValue["dF"] = distanceFront;
	sendValue["dL"] = distanceLeft;
	sendValue["dR"] = distanceRight;
	sendValue["aP"] = autopilot;
	sendValue["bV"] = batteryVoltage;
	sendValue["eV"] = engineBatteryVoltage;
	sendValue["eB"] = emergencyButton ? 1 : 0;

	Serial.flush();
	while(!Serial.availableForWrite()) {}

	sendValue.printTo(Serial);
	Serial.println();
	Serial.send_now();
}

inline void updateValues() {
	// Read actuator values
	brakePotentiometer = brakePotentiometerRead() / 64.0f;
	steeringEncoder = steeringEncoderRead();
	steeringPotentiometer = steeringPotentiometerRead() / 64.0f;
	batteryVoltage = batteryVoltageRead();
	emergencyButton = emergencyButtonRead();
	if(millis() - i2cTiming >= I2C_PERIOD) {
		// Read LIDAR Lite values
		if(isLeftLidarConnected) distanceLeft = leftLidarLite.distance(true);
		else distanceLeft = -1;
		delay(1);
		if(isRightLidarConnected) distanceRight = rightLidarLite.distance(true);
		else distanceRight = -1;
		i2cTiming = millis();
	}
	//if(isFrontDistanceSensorConnected) distanceFront = distanceRight; // TODO For Sonar: distanceFrontRead();
	//else distanceFront = -1;
	// Read following values from CANBUS
	speedWheel = CAN.getMeasurement(ECU_SPEED);
	brakePressure1 = CAN.getMeasurement(FRONT_BRAKE);
	brakePressure2 = CAN.getMeasurement(ECU_BRAKE);
	gear = CAN.getMeasurement(ECU_GEAR);
	distanceWheel = CAN.getMeasurement(ECU_DISTANCE);
	autopilot = CAN.getSystemState(STEERING_ID, BUTTON_AUTOPILOT);
	engineBatteryVoltage = CAN.getMeasurement(ECU_BAT_VOLTAGE);

	// Trim values
	batteryVoltage = ((int)(batteryVoltage * 10 + .5) / 10.0);
	engineBatteryVoltage = ((int)(engineBatteryVoltage * 10 + .5) / 10.0);

	// Set autopilot active pin
	//autoState(autopilotActive);	// TODO
}

void loop() {
	bool success = read();
	if(!success) return;

	updateValues();

	send();
	LED1Toggle();

	if((millis() - canTiming) >= CAN_SEND_PERIOD) {
		canTiming = millis();
		// Disable autopilotActive signal when emergency button pressed
		if(!emergencyButton) autopilotActive = 0;
		// Send status on CAN
		if(millis() - lastSerialReceivedTime <= SERIAL_TIMEOUT_MS) {
			CAN.sendStatus(CAN_AUTO_PRIORITY,
				(burn << AUTO_BURN |
					horn << AUTO_HORN |
					light << AUTO_LIGHT |
					blinkLeft << AUTO_BLINK_LEFT |
					blinkRight << AUTO_BLINK_RIGHT |
					autopilotActive << AUTO_HAS_CONTROL |
					parking << AUTO_PARKING));
		} else {
			// Timeout(too long since last message received)
			CAN.sendStatus(CAN_AUTO_PRIORITY, 0);
		}
		CAN.sendMeasurement(CAN_AUTO_PRIORITY, 0, AUTO_DISTANCE_FRONT, distanceFront);
		CAN.sendMeasurement(CAN_AUTO_PRIORITY, 0, AUTO_STEERING_POT, steeringPotentiometer);
		CAN.sendMeasurement(CAN_AUTO_PRIORITY, 0, AUTO_BATTERY_VOLTAGE, batteryVoltage);
	}
	// END LOOP */
}

void watchdogCallback() {
	// WD Timeout (too long since last message received)
	if(millis() - lastSerialReceivedTime > SERIAL_TIMEOUT_MS) {
		lastWatchdogActionTime = millis();
		autopilotActive = 0;
	}
}
