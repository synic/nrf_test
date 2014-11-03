#ifndef _AOLIB_H_
#define _AOLIB_H_

#include <stdint.h>

#define DELAY_TIM_FREQUENCY_US 1000000		/* = 1MHZ -> timer runs in microseconds */
#define DELAY_TIM_FREQUENCY_MS 1000			/* = 1kHZ -> timer runs in milliseconds */

static volatile uint32_t counter;
static volatile uint32_t timing_delay;

extern void delay_ms(uint32_t delay);
extern void delay_us(uint32_t delay);
extern void systick_handler(void);
extern void delay_init(void);

uint32_t millis(void);

#endif
