#ifndef _AOLIB_H_
#define _AOLIB_H_

#include <stdint.h>

void delay(volatile uint64_t delay);
void delay_us(volatile uint64_t delay);
uint64_t millis();

#endif
