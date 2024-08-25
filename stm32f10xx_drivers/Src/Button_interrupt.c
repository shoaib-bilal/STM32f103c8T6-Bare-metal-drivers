/*
 * Button_interrupt.c
 *
 *  Created on: 13-Apr-2024
 *      Author: shoai
 */


#include <string.h>
#include <stdint.h>
#include "stm32f103xx.h"
#include "stm32f103xx_gpio_driver.h"


void delay()
{
	for(uint32_t i=0; i<500000/2; i++);
}

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

int main(void)
{
    /* Loop forever */


	GPIO_Handle_t GpioLed,GpioButton,GpioIn;
	memset(&GpioLed,0, sizeof(GpioLed));
	memset(&GpioButton,0,sizeof(GpioButton));
	memset(&GpioIn,0,sizeof(GpioIn));

	GpioLed.pGPIOx = GPIOC;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_13;
	GpioLed.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_OUT_50Mhz;
	GpioLed.GPIO_PinConfig.GPIO_PinOpType= GPIO_OP_TYPE_PUSHPULL;



	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_0;
	GpioButton.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_IT_FT;
	//GpioButton.pGPIOx->ODR |= 1<< (GPIO_PIN_NO_0);


	//GpioIn.pGPIOx=GPIOA;
	//GpioIn.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_0;
	//GpioIn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	//GpioIn.GPIO_PinConfig.GPIO_PinInType=GPIO_IN_TYPE_PULLUP;



	//GpioLed.GPIO_PinConfig.GPIO_PinInType= GPIO_IN_TYPE_PULLUP;

	GPIOC_PCLK_EN();
//	GPIOA_PCLK_EN();


	//GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRIO15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0,ENABLE);

	//GPIO_Init(&GpioIn);
	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioButton);



	while(1);
}

void EXTI0_IRQHandler(void)
	{
		//delay();
		GPIO_IRQHandling(GPIO_PIN_NO_0);

		for(uint32_t i=0; i<5; i++)
		{
		GPIO_ToggleOutputPin(GPIOC,GPIO_PIN_NO_13);
		delay();
		}



	}
