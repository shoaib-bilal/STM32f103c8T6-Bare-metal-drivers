/*
 * stm32f103xx_gpio_driver.c
 *
 *  Created on: 22-Jan-2024
 *      Author: shoai
 */


#include "stm32f103xx_gpio_driver.h"


/*
 * Peripheral clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi )
{
	if(EnorDi== ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
	}
	else
	{


	}

}




/*
 * Init and DeInit
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode > GPIO_MODE_IN)
	{

		/* GPIO_MODE_IN				0
		 * GPIO_MODE_OUT_10MHz		1
		 * GPIO_MODE_OUT_2MHz		3
 	 	 * GPIO_MODE_OUT_50MHz		4
 	 	 *
 	 	 * ------Code for output mode----------
		 */
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber<=7)
		{	/*
			 * Code for output mode if pin is 0-7
			 */
			uint32_t temp1=0;
			uint32_t temp2=0;

			//Code to configure required pin as output and its speed (10/2/50MHz)
			temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<< (4* (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->CRL &=~ (0x03<< (4* (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
			pGPIOHandle->pGPIOx->CRL |=temp1;

			//Code to configure required pin's output mode type
			/* PUSHPULL
			 * OPENDRAIN
			 * AL_PUSHPULL
			 * AL_OPENDRAIN
			 */
			temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinOpType<<(2+ (4* (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
			pGPIOHandle->pGPIOx->CRL &= ~ (0x03 << (2+ (4* (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber))));
			pGPIOHandle->pGPIOx->CRL |=temp2;
		}

		else
		{	/*
			 * Code for output mode if pin is 8-15
			 */
			uint32_t temp1=0;
			uint32_t temp2=0;

			//clear 20 and 21st bit
			//*pPortCconfighighreg &= ~(3<<20);

			//set 20 and 21st bit for output mode 50Mhz
		//	*pPortCconfighighreg |= 3<<20;

			//clear 22 and 23 bit for general purpose o/p push pull
		//	*pPortCconfighighreg &= ~(3<<22) ;
			//Code to configure requir	`ed pin as output and its speed
			/* 10MHz
			 * 2MHz
			 * 5MHz
			 */
			temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<< (4* ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)-8));
			pGPIOHandle->pGPIOx->CRH &= ~( 0x03<< (4* ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)-8)));
			pGPIOHandle->pGPIOx->CRH |=temp1;

			//Code to configure required pin's output mode type
			/* PUSHPULL
			 * OPENDRAIN
			 * AL_PUSHPULL
			 * AL_OPENDRAIN
			 */
			temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinOpType<<(2+(4* ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)-8)));
			pGPIOHandle->pGPIOx->CRH &= ~ (0x03 << (2+(4* ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)-8))));
			pGPIOHandle->pGPIOx->CRH |= temp2;
		}

	}


	else
	{
		/*
		 * GPIO_MODE_IN			0
		 *
		 *------ Code for Input mode-------------
		 */
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber<=7)
		{
			/*
			 * Code for Input mode if pin is 0-7
			 */
			uint32_t temp1=0;
			uint32_t temp2=0;

			temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << ( 4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->CRL &= ~(0x03 << ( 4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
			pGPIOHandle->pGPIOx->CRL |= temp1;

			/*
			 *
			 * Code to configure required input pin type
			 * GPIO_IN_TYPE_ANALOG		0
			 * GPIO_IN_TYPE_FLOATING	1
			 * GPIO_IN_TYPE_PULLUP		2
			 * GPIO_IN_TYPE_PULLDOWN	3
			 */

			if(pGPIOHandle->GPIO_PinConfig.GPIO_PinInType== GPIO_IN_TYPE_PULLUP)
			{
				pGPIOHandle->pGPIOx->ODR |= 1<< (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
				temp2 = 2 << (2+ (4* (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
				pGPIOHandle->pGPIOx->CRL &= ~ ( 0x03 << (2+ (4* (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber))));
				pGPIOHandle->pGPIOx->CRL |= temp2;

			}
			else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinInType== GPIO_IN_TYPE_PULLDOWN)
			{
				pGPIOHandle->pGPIOx->ODR &= ~ ( 1<< (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
				temp2 = 2 << (2+ (4* (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
				pGPIOHandle->pGPIOx->CRL &= ~ ( 0x03 << (2+ (4* (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber))));
				pGPIOHandle->pGPIOx->CRL |= temp2;
			}
			else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinInType== (GPIO_IN_TYPE_ANALOG || GPIO_IN_TYPE_FLOATING ))
			{
			temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinInType << (2+ (4* (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
			pGPIOHandle->pGPIOx->CRL &= ~ (0x03 << (2+ (4* (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber))));
			pGPIOHandle->pGPIOx->CRL |= temp2;
			}
		}
		else
		{
			/*
			 *Code for Input mode if pin 8-15
			 */
			uint32_t temp1=0;
			uint32_t temp2=0;

			temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << ( 4 * ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)-8));
			pGPIOHandle->pGPIOx->CRH &=~ ( 0x03 << ( 4 * ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)-8)));
			pGPIOHandle->pGPIOx->CRH |= temp1;

			/*
			 *
			 * Code to configure required input pin type
			 * GPIO_IN_TYPE_ANALOG		0
			 * GPIO_IN_TYPE_FLOATING	1
			 * GPIO_IN_TYPE_PULLUP		2
			 * GPIO_IN_TYPE_PULLDOWN	3
			 */
			if(pGPIOHandle->GPIO_PinConfig.GPIO_PinInType== GPIO_IN_TYPE_PULLUP)
			{
				pGPIOHandle->pGPIOx->ODR |= 1<< (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
				temp2 = 2<< ( 2 + (4 * ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)-8)));
				pGPIOHandle->pGPIOx->CRH &= ~ (0x03 << ( 2 + (4 * ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)-8))));
				pGPIOHandle->pGPIOx->CRH |= temp2;
			}
			else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinInType== GPIO_IN_TYPE_PULLDOWN)
			{
				pGPIOHandle->pGPIOx->ODR &= ~ ( 1<< (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
				temp2= 2 << ( 2 + (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)-8));
				pGPIOHandle->pGPIOx->CRH &=~ ( 0x03 << ( 2 + (4 * ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)-8))));
				pGPIOHandle->pGPIOx->CRH |= temp2;
			}
			else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinInType== (GPIO_IN_TYPE_ANALOG || GPIO_IN_TYPE_FLOATING ))
			{
				temp2= pGPIOHandle->GPIO_PinConfig.GPIO_PinOpType << ( 2 + (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)-8));
				pGPIOHandle->pGPIOx->CRH &=~ ( 0x03 << ( 2 + (4 * ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)-8))));
				pGPIOHandle->pGPIOx->CRH |= temp2;
			}
		}

	}
}



void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DN();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DN();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DN();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DN();
		}


}


/*
 * Data Read write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
uint8_t value= (uint8_t) ((pGPIOx->IDR >>PinNumber) & (0x1));

return value;
}


uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value= (uint16_t) pGPIOx->IDR;
	return value;
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
	if (value== GPIO_PIN_SET)
	{
		pGPIOx->ODR |= 1<< PinNumber;
	}
	else
	{
		pGPIOx->ODR &= ~ (1<<PinNumber);
	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t value)
{
	pGPIOx->ODR |=value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1<<PinNumber);
}

/*
 * IRQ configuration and IRS handling
 */
//void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
//void GPIO_IRQHandling(uint8_t PinNumber);
