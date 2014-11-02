#include "stm32f30x.h"
#include "main.h"
#include "aolib.h"
#include "rf24.h"

volatile uint64_t counter = 0;
volatile uint64_t _delay_us = 0;
volatile uint64_t delay_ms = 0;

uint64_t s_t = sizeof(uint64_t) - 2;

void systick_handler(void) {
    counter++;
    if(counter >= s_t) counter = 0;
    if(_delay_us > 0) {
        _delay_us--;

        if(_delay_us % 1000 == 0 && delay_ms > 0) {
            delay_ms--;
        }
    }
}


int main(void) {
    uint8_t loops = 0;

    if(SysTick_Config(SystemCoreClock / 1000000L)) {
        while(1) { }
    }

    return 0;
}

