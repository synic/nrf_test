#include "aolib.h"
#include "main.h"
#include <stdint.h>

void delay(volatile uint64_t delay) {
    delay_ms = delay;
    while(delay_ms > 0);
}

void delay_us(volatile uint64_t delay) {
    _delay_us = delay;
    while(_delay_us > 0);
}

uint64_t millis() {
    return counter / 1000;
}
