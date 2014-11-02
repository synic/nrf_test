#include "stm32f30x.h"
#include "main.h"
#include "aolib.h"
#include "rf24.h"

RF24 radio;

const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };
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
    if(SysTick_Config(SystemCoreClock / 1000000L)) {
        while(1) { }
    }

    radio.begin();
    radio.setRetries(15, 15);

    radio.openReadingPipe(1, pipes[1]);
    radio.startListening();

    while(1) {
        radio.stopListening();
        unsigned long time = millis();
        bool ok = radio.write(&time, sizeof(unsigned long));

        if(ok) {
        }
        else {
            while(1); // definitely not ok.
        }

        unsigned long started_waiting_at = millis();
        bool timeout = false;
        while(!radio.available() && !timeout) {
            if(millis() - started_waiting_at > 200) {
                timeout = true;
            }
        }

        delay(1000);
    }

    return 0;
}
