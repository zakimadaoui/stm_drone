/*
 * micros.c
 *
 *  Created on: Oct 4, 2021
 *      Author: zaki
 */

#include "micros.h"


void micros_init(){

	// Enable clock for that module for TIM3. Bit1 in RCC APB1ENR register
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN_Msk;

	// CONFIG USING COTROL REGISTER 1 TO:
	TIM3->CR1 = 0; // Reset CR1 just in case
	TIM3->CR1 |= TIM_CR1_ARPE_Msk; // enable auto-reload preload
	TIM3->CR1 |= TIM_CR1_URS_Msk; // only overflow generates event
	
	// set prescaler to no division
	TIM3->PSC = 0;
	
	// Set the auto-reload value to count from 0 to 719 (div clock by 720)
	// which should generate 10 uS update events, ie 0.1MHz interrupts
	// TIM3->ARR = 719; 
	
	// Set the auto-reload value to count from 0 to 287 (div clock by 288)
	// which should generate 4 uS update events, ie 0.1MHz interrupts
	TIM3->ARR = 287; 
	// Update Interrupt Enable
	TIM3->DIER |= TIM_DIER_UIE_Msk;

	// Priority level 3
	NVIC_SetPriority(TIM3_IRQn, 0x03); //TODO: try level 0 (Max priority)
	// Enable TIM3 from NVIC register
	NVIC_EnableIRQ(TIM3_IRQn);

	// Finally enable TIM3 module
	TIM3->CR1 |= TIM_CR1_CEN_Msk;

}

uint32_t micros(){
	return uu_ticks;
}

void TIM3_IRQHandler(void){
	// check the update event flag "UIF" 
	if((TIM3->SR & TIM_SR_UIF_Msk) == TIM_SR_UIF_Msk){
		// uu_ticks+=10; // increment each 10 Micro-seconds
		uu_ticks+=4; // increment each 4 Micro-seconds
		TIM3->SR &= ~TIM_SR_UIF_Msk;
	}
}
