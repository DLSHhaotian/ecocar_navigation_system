// led.h

#ifndef _LED_h
#define _LED_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

void LED_init();

void LED1On(void);
void LED2On(void);

void LED1Off(void);
void LED2Off(void);

void LED1Toggle(void);
void LED2Toggle(void);

void LEDBlink(int LED, int times);

#endif

