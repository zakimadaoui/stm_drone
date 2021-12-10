/*
 * micros.h
 *
 *  Created on: Oct 4, 2021
 *      Author: zaki
 */

#ifndef INC_MICROS_H_
#define INC_MICROS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f1xx.h"


volatile static uint32_t uu_ticks = 0; //don't forget the "volatile" keyword !!!

void micros_init(); //config and start timer for 1us ticks
uint32_t micros(); //get the current tick (MAX is 1h and few mins currently !)
void TIM3_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_MICROS_H_ */
