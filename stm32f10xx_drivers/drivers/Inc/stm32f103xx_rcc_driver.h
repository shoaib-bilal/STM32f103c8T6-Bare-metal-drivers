/*
 * stm32f103xx_rcc_driver.h
 *
 *  Created on: 17-Jul-2024
 *      Author: shoai
 */

#ifndef INC_STM32F103XX_RCC_DRIVER_H_
#define INC_STM32F103XX_RCC_DRIVER_H_


#include"stm32f103xx.h"

 //This returns value of APB1 clock
uint32_t RCC_GetPCLK1Value(void);

//This returns value of APB2 clock
uint32_t RCC_GetPCLK2Value(void);

uint32_t RCC_GetPLLOutputClock(void);


#endif /* INC_STM32F103XX_RCC_DRIVER_H_ */
