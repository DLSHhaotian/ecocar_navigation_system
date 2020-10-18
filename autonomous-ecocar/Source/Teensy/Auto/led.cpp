// 
// 
// 

#include "led.h"

#define LED1 33		// Green
#define LED2 39		// Blue

void LED_init(void) {
	pinMode(LED1, OUTPUT);
	pinMode(LED2, OUTPUT);
}

// LED 1
void LED1On(void) {
	digitalWriteFast(LED1, HIGH);
}
void LED1Off(void) {
	digitalWriteFast(LED1, LOW);
}
void LED1Toggle(void) {
	digitalWrite(LED1, !digitalReadFast(LED1));
}


// LED 2
void LED2On(void) {
	digitalWriteFast(LED2, HIGH);
}
void LED2Off(void) {
	digitalWriteFast(LED2, LOW);
}
void LED2Toggle(void) {
	digitalWrite(LED2, !digitalReadFast(LED2));
}

void LEDBlink(int LED, int times) {
	for(int i = 0; i < times; i++) {
		switch(LED) {
		case 1:
			LED1On();
			delay(200);
			LED1Off();
			delay(200);
			break;
		case 2:
			LED2On();
			delay(200);
			LED2Off();
			delay(200);
			break;
		}
	}
}