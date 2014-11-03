#include "stm32f30x.h"
#include "aolib.h"
#include "rf24.h"

RF24 radio;

const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

int main(void) {
    bool outmode = false;
    delay_init();

    radio.begin();
    radio.setRetries(15, 15);
    radio.setPayloadSize(8);
    radio.setDataRate(RF24_250KBPS);

    if(outmode) {
        radio.openWritingPipe(pipes[0]);
        radio.openReadingPipe(1, pipes[1]);
    }
    else {
        radio.openWritingPipe(pipes[1]);
        radio.openReadingPipe(1, pipes[0]);
    }

    radio.startListening();

    while(1) {
        if(outmode) { // ping out mode
            radio.stopListening();
            unsigned long time = millis();
            bool ok = radio.write(&time, sizeof(unsigned long));

            radio.startListening();
            unsigned long started_waiting_at = millis();
            bool timeout = false;
            while(!radio.available() && !timeout) {
                if(millis() - started_waiting_at > 200) {
                    timeout = true;
                }
            }

            if(!timeout) {
                unsigned long got_time;
                radio.read(&got_time, sizeof(unsigned long));
            }

            delay_ms(1000);
        }
        else { // receive and pong back mode
            if(radio.available()) {
                unsigned long got_time;
                bool done = false;
                while(!done) {
                    done = radio.read(&got_time, sizeof(unsigned long));
                    delay_ms(20);
                }

                radio.stopListening();
                radio.write(&got_time, sizeof(unsigned long));
                radio.startListening();
            }
        }
    } 

    return 0;
}
