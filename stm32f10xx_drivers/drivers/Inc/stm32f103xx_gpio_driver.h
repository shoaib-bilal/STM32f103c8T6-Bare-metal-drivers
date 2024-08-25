/*
 * stm32f103xx_gpio_driver.h
 *
 *  Created on: 22-Jan-2024
 *      Author: shoai
 */

#ifndef INC_STM32F103XX_GPIO_DRIVER_H_
#define INC_STM32F103XX_GPIO_DRIVER_H_



#include "stm32f103xx.h"


typedef struct
{
	uint32_t GPIO_PinNumber;
	uint32_t GPIO_PinMode;
	uint32_t GPIO_PinOpType;
	uint32_t GPIO_PinInType;
	uint32_t GPIO_PinAltFunMode;


}GPIO_PinConfig_t;


/*
 * This is a handle structure for GPIO Pin
 */

typedef struct
{
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;


/*
 * GPIO Pin Numbers
 */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15



/*
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN			0
#define GPIO_MODE_OUT_10Mhz		1
#define GPIO_MODE_OUT_2Mhz		2
#define GPIO_MODE_OUT_50Mhz		3
#define GPIO_MODE_IT_FT			4
#define GPIO_MODE_IT_RT			5
#define GPIO_MODE_IT_RFT		6


/*
 * GPIO pin possible input types
 */

#define GPIO_IN_TYPE_ANALOG		0
#define GPIO_IN_TYPE_FLOATING	1
#define GPIO_IN_TYPE_PULLUP		2
#define GPIO_IN_TYPE_PULLDOWN	3


/*
 * GPIO pin possible output types
 */

#define GPIO_OP_TYPE_PUSHPULL	0
#define GPIO_OP_TYPE_OPENDRAIN	1
#define GPIO_OP_AL_PUSHPULL		2
#define GPIO_OP_AL_OPENDRAIN	3




/**********************APIs supported by this driver****************************************/
/*
 * For more information about the API check function definition
 ******************************************************************************************/


/*
 * Peripheral clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi );

/*
 * Init and DeInit
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


/*
 * Data Read write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);


/*
 * IRQ configuration and IRS handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F103XX_GPIO_DRIVER_H_ */
