#include "aolib.h"
#include "stm32f30x.h"
#include <stdint.h>

void delay_init(void) {
    SysTick_Config(SystemCoreClock / DELAY_TIM_FREQUENCY_MS);
}

void _init_us() {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	// Time base configuration
	TIM_TimeBaseInitTypeDef TIM;
	TIM_TimeBaseStructInit(&TIM);
	TIM.TIM_Prescaler = (SystemCoreClock/DELAY_TIM_FREQUENCY_US)-1;
	TIM.TIM_Period = UINT16_MAX;
	TIM.TIM_ClockDivision = 0;
	TIM.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM);

	// Enable counter for TIM2
	TIM_Cmd(TIM2, ENABLE);
}

// Init and start timer for Milliseconds delays
void _init_ms() {
	// Enable clock for TIM2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);

	// Time base configuration
	TIM_TimeBaseInitTypeDef TIM;
	TIM_TimeBaseStructInit(&TIM);
	TIM.TIM_Prescaler = (SystemCoreClock/DELAY_TIM_FREQUENCY_MS)-1;
	TIM.TIM_Period = UINT16_MAX;
	TIM.TIM_ClockDivision = 0;
	TIM.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM);

	// Enable counter for TIM2
	TIM_Cmd(TIM2, ENABLE);
}

// Stop timer
void _stop_timer() {
	TIM_Cmd(TIM2 ,DISABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, DISABLE); // Powersavings?
}

// Do delay for nTime milliseconds
void delay_ms(uint32_t mSecs) {
	// Init and start timer
	_init_ms();

	// Dummy loop with 16 bit count wrap around
	volatile uint32_t start = TIM2->CNT;
	while((TIM2->CNT-start) <= mSecs);

	// Stop timer
	_stop_timer();
}

// Do delay for nTime microseconds
void delay_us(uint32_t uSecs) {
	// Init and start timer
	_init_us();

	// Dummy loop with 16 bit count wrap around
	volatile uint32_t start = TIM2->CNT;
	while((TIM2->CNT-start) <= uSecs);

	// Stop timer
	_stop_timer();
}

uint32_t millis() {
    return counter;
}

void systick_handler(void) {
    counter++;
}
