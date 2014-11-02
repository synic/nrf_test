#ifndef __NEOCLOCK_MAIN_H
#define __NEOCLOCK_MAIN_H

extern volatile uint64_t counter;
extern volatile uint64_t delay_ms;
extern volatile uint64_t _delay_us;

void systick_handler(void);
int main(void);

#endif
