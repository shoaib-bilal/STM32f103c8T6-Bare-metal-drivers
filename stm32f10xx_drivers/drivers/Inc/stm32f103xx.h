/*
 * stm32f103xx.h
 *
 *  Created on: Jan 15, 2024
 *      Author: shoai
 */

#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_

#include<stddef.h>
#include<stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))

/***************************START:Processor specific details*************************/

/*
  * ARM Cortex Mx processor NVIC ISERx register Address
  */

#define NVIC_ISER0			((__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1			((__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2			((__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3			((__vo uint32_t*)0xE000E10C )

/*
  * ARM Cortex Mx processor NVIC ICERx register Address
  */

#define NVIC_ICER0			((__vo uint32_t*)0XE000E180 )
#define NVIC_ICER1			((__vo uint32_t*)0XE000E184 )
#define NVIC_ICER2			((__vo uint32_t*)0XE000E188 )
#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C )


/*
 * ARM Cortex Mx processor NVIC IPRx register Address calculation
 */
#define NVIC_PR_BASE_ADDR	((uint32_t*)0xE000E400 )

/*
 * ARM Cortex Mx processor No. f priority bits implemented in priority register
 */
#define NO_PR_BITS_IMPLEMENTED		4



/*
Base address of Flash and SRAM memories
*/
#define FLASH_BASEADDR			0x08000000U
#define SRAM					0x20000000U
#define ROM_BASEADDR			0x1FFFF000U


/*
 * AHBx and APBx Bus Base address
 */
#define PERIPH_BASE				0x40000000U
#define AHBPERIPH_BASE 			0x40018000U
#define APB1PERIPH_BASE			PERIPH_BASE
#define APB2PERIPH_BASE			0x40010000U


/*
 * Base address of peripherals which are hanging on APB2 bus
 */
#define AFIO_BASEADDR			(APB2PERIPH_BASE + 0x0000)

#define EXTI_BASEADDR			(APB2PERIPH_BASE + 0x0400)

#define GPIOA_BASEADDR 			(APB2PERIPH_BASE + 0x0800)
#define GPIOB_BASEADDR 			(APB2PERIPH_BASE + 0x0C00)
#define GPIOC_BASEADDR 			(APB2PERIPH_BASE + 0x1000)
#define GPIOD_BASEADDR 			(APB2PERIPH_BASE + 0x1400)
#define GPIOE_BASEADDR 			(APB2PERIPH_BASE + 0x1800)
#define GPIOF_BASEADDR 			(APB2PERIPH_BASE + 0x1C00)
#define GPIOG_BASEADDR 			(APB2PERIPH_BASE + 0x2000)


#define ADC1_BASEADDR			(APB2PERIPH_BASE + 0x2400)
#define ADC2_BASEADDR			(APB2PERIPH_BASE + 0x2800)
#define TIM1_BASEADDR			(APB2PERIPH_BASE + 0x2C00)
#define SPI1_BASEADDR			(APB2PERIPH_BASE + 0x3000)
#define TIM8_BASEADDR			(APB2PERIPH_BASE + 0x3400)
#define USART1_BASEADDR			(APB2PERIPH_BASE + 0x3800)
#define ADC3_BASEADDR			(APB2PERIPH_BASE + 0x3C00)
#define RESERVED1_BASEADDR		(APB2PERIPH_BASE + 0x4000)
#define TIM9_BASEADDR			(APB2PERIPH_BASE + 0x4C00)
#define TIM10_BASEADDR			(APB2PERIPH_BASE + 0x5000)
#define TIM11_BASEADDR			(APB2PERIPH_BASE + 0x5400)
#define RESERVED2_BASEADDR		(APB2PERIPH_BASE + 0x5800)


/*
 * Base address of peripheral which are hanging on APB1 bus
 */
#define TIM2_BASEADDR			(APB1PERIPH_BASE + 0x0000)
#define TIM3_BASEADDR			(APB1PERIPH_BASE + 0x0400)
#define TIM4_BASEADDR			(APB1PERIPH_BASE + 0x0800)
#define TIM5_BASEADDR			(APB1PERIPH_BASE + 0x0C00)
#define TIM6_BASEADDR			(APB1PERIPH_BASE + 0x1000)
#define TIM7_BASEADDR			(APB1PERIPH_BASE + 0x1400)
#define TIM12_BASEADDR			(APB1PERIPH_BASE + 0x1800)
#define TIM13_BASEADDR			(APB1PERIPH_BASE + 0x1C00)
#define TIM14_BASEADDR			(APB1PERIPH_BASE + 0x2000)

#define RTC_BASEADDR			(APB1PERIPH_BASE + 0x2800)
#define WWDG_BASEADDR			(APB1PERIPH_BASE + 0x2C00)
#define IWDG_BASEADDR			(APB1PERIPH_BASE + 0x3000)

#define SPI2_BASEADDR			(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASE + 0x3C00)

#define USART2_BASEADDR			(APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR			(APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR			(APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR			(APB1PERIPH_BASE + 0x5000)

#define I2C1_BASEADDR			(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASE + 0x5800)

#define CAN2_BASEADDR			(APB1PERIPH_BASE + 0x6800)
#define CAN1_BASEADDR			(APB1PERIPH_BASE + 0x6400)

#define BKP_BASEADDR			(APB1PERIPH_BASE + 0x6C00)
#define PWR_BASEADDR			(APB1PERIPH_BASE + 0x7000)
#define DAC_BASEADDR			(APB1PERIPH_BASE + 0x7400)


/*
 * Base address of peripheral which are hanging on AHB bus
 */
#define SDIO_BASEADDR			(AHBPERIPH_BASE + 0x0000)

#define DMA1_BASEADDR			(AHBPERIPH_BASE + 0x8000 + 0x0400)
#define DMA2_BASEADDR			(AHBPERIPH_BASE + 0x8000 + 0x0800)
#define RCC_BASEADDR			(AHBPERIPH_BASE + 0x8000 + 0x1000)

#define FLASH_INTERFACE_BASEADDR		(AHBPERIPH_BASE + 0x8000 + 0x2000)

#define CRC_BASEADDR			(AHBPERIPH_BASE + 0x8000 + 0x3000)
#define ETHERNET_BASEADDR		(AHBPERIPH_BASE + 0x8000 + 0x8000)
//#define USB_OTG_BASEADDR		(AHBPERIPH_BASE + 0x8000 + 0x4000)
//#define FSMC_BASEADDR			(AHBPERIPH_BASE + 0x8000 + 0x4000)




/******************************Peripheral register definition structure************************/
/*
 * Note: Registers of Peripheral are specific to MCU
 */

typedef struct
{
	__vo uint32_t CRL;
	__vo uint32_t CRH;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t BRR;
	__vo uint32_t LCKR;
}GPIO_RegDef_t;



/*
 * RCC peripheral register definition
 */
typedef struct
{

	__vo uint32_t RCC_CR;
	__vo uint32_t RCC_CFGR;
	__vo uint32_t RCC_CIR;
	__vo uint32_t RCC_APB2RSTR;
	__vo uint32_t RCC_APB1RSTR;
	__vo uint32_t RCC_AHBENR;
	__vo uint32_t RCC_APB2ENR;
	__vo uint32_t RCC_APB1ENR;
	__vo uint32_t RCC_BDCR;
	__vo uint32_t RCC_CSR;

}RCC_RegDef_t;


/*
 * EXTI register definition
 */
typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;

}EXTI_RegDef_t;

/*
 * SPI register definition
 */
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;

}SPI_RegDef_t;


/*
 * AFIO register definition
 */
typedef struct
{
	__vo uint32_t EVCR;
	__vo uint32_t MAPR;
	__vo uint32_t EXTICR[4];
	__vo uint32_t MAPR2;

}AFIO_RegDef_t;

/*
 * I2C register definition
 */
typedef struct
{
	__vo uint32_t	CR1;
	__vo uint32_t	CR2;
	__vo uint32_t	OAR1;
	__vo uint32_t	OAR2;
	__vo uint32_t	DR;
	__vo uint32_t	SR1;
	__vo uint32_t	SR2;
	__vo uint32_t	CCR;
	__vo uint32_t	TRISE;

}I2C_RegDef_t;


/*
 * USART register definition
 */
typedef struct
{
	__vo uint32_t	SR;
	__vo uint32_t	DR;
	__vo uint32_t	BRR;
	__vo uint32_t	CR1;
	__vo uint32_t	CR2;
	__vo uint32_t	CR3;
	__vo uint32_t	GTPR;


}USART_RegDef_t;
/*
 * Peripheral definition( Peripheral base address type casted to xxxx_RegDef_t)
 */
#define GPIOA 		((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD		((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE		((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF		((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG		((GPIO_RegDef_t*)GPIOG_BASEADDR)

#define RCC			((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI		((EXTI_RegDef_t*)EXTI_BASEADDR)
#define AFIO		((AFIO_RegDef_t*)AFIO_BASEADDR)

#define SPI1		((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2		((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3		((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C1		((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2		((I2C_RegDef_t*)I2C2_BASEADDR)

#define USART1		((USART_RegDef_t*)USART1_BASEADDR)
#define USART2		((USART_RegDef_t*)USART2_BASEADDR)
#define USART3		((USART_RegDef_t*)USART3_BASEADDR)


/*
 * Clock enable macros for GPIOx peripheral
 */
#define GPIOA_PCLK_EN()		RCC->RCC_APB2ENR |= (1<<2)
#define GPIOB_PCLK_EN()		RCC->RCC_APB2ENR |= (1<<3)
#define GPIOC_PCLK_EN()		RCC->RCC_APB2ENR |= (1<<4)
#define GPIOD_PCLK_EN()		RCC->RCC_APB2ENR |= (1<<5)
#define GPIOE_PCLK_EN()		RCC->RCC_APB2ENR |= (1<<6)

#define AFIO_PCLK_EN()		RCC->RCC_APB2ENR |= (1<<0)

#define SPI1_PCLK_EN()		RCC->RCC_APB2ENR |= (1<<12)
#define SPI2_PCLK_EN()		RCC->RCC_APB1ENR |= (1<<14)
#define SPI3_PCLK_EN()		RCC->RCC_APB1ENR |= (1<<15)

#define I2C1_PCLK_EN()		RCC->RCC_APB1ENR |= (1<<21)
#define I2C2_PCLK_EN()		RCC->RCC_APB1ENR |= (1<<22)

#define USART1_PCLK_EN()	RCC->RCC_APB2ENR |= (1<<14)
#define USART2_PCLK_EN()	RCC->RCC_APB1ENR |= (1<<17)
#define USART3_PCLK_EN()	RCC->RCC_APB1ENR |= (1<<18)


/*
 * Clock Disable macros for GPIOx peripheral
 */
#define GPIOA_PCLK_DN()		do{ (RCC->RCC_APB2RSTR |= (1<<2)); (RCC->RCC_APB2RSTR &=~ (1<<2));} while(0)
#define GPIOB_PCLK_DN()		do{ (RCC->RCC_APB2RSTR |= (1<<3)); (RCC->RCC_APB2RSTR &=~ (1<<3));} while(0)
#define GPIOC_PCLK_DN()		do{ (RCC->RCC_APB2RSTR |= (1<<4)); (RCC->RCC_APB2RSTR &=~ (1<<4));} while(0)
#define GPIOD_PCLK_DN()		do{ (RCC->RCC_APB2RSTR |= (1<<5)); (RCC->RCC_APB2RSTR &=~ (1<<5));} while(0)
#define GPIOE_PCLK_DN()		do{ (RCC->RCC_APB2RSTR |= (1<<6)); (RCC->RCC_APB2RSTR &=~ (1<<6));} while(0)

#define AFIO_PCLK_DN()		do{ (RCC->RCC_APB2RSTR |= (1<<0)); (RCC->RCC_APB2RSTR &=~ (1<<0));} while(0)

#define SPI1_PCLK_DN()		do{ (RCC->RCC_APB2RSTR |= (1<<12)); (RCC->RCC_APB2RSTR &=~ (1<<12));} while(0)
#define SPI2_PCLK_DN()		do{ (RCC->RCC_APB1RSTR |= (1<<14)); (RCC->RCC_APB1RSTR &=~ (1<<14));} while(0)
#define SPI3_PCLK_DN()		do{ (RCC->RCC_APB1RSTR |= (1<<15)); (RCC->RCC_APB1RSTR &=~ (1<<15));} while(0)

#define I2C1_PCLK_DN()		do{ (RCC->RCC_APB1RSTR |= (1<<21)); (RCC->RCC_APB1RSTR &=~ (1<<21));} while(0)
#define I2C2_PCLK_DN()		do{ (RCC->RCC_APB1RSTR |= (1<<22)); (RCC->RCC_APB1RSTR &=~ (1<<22));} while(0)

#define USART1_PCLK_DN()	do{ (RCC->RCC_APB2RSTR |= (1<<14)); (RCC->RCC_APB2RSTR &=~ (1<<14));} while(0)
#define USART2_PCLK_DN()	do{ (RCC->RCC_APB1RSTR |= (1<<17));	(RCC->RCC_APB1RSTR &=~ (1<<17));} while(0)
#define USART3_PCLK_DN()	do{ (RCC->RCC_APB1RSTR |= (1<<18)); (RCC->RCC_APB1RSTR &=~ (1<<18));} while(0)

/*
 * Returns port code for given GPIOx base address
 */
#define GPIO_BASEADDR_TO_CODE(x)	((x==GPIOA)?0:\
									 (x==GPIOB)?1:\
									 (x==GPIOC)?2:\
									 (x==GPIOD)?3:\
									 (x==GPIOE)?4:\
									 (x==GPIOF)?5:\
									 (x==GPIOG)?6:0)

/*
 * Interrupt request no of STM32F10xxx MCU
 * Note: Update these macros with valid values according to your MCU
 * Todoo: You may complete this list for other peripherals
 */

#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_I2C1_EV		31
#define IRQ_NO_I2C1_ER		32
#define IRQ_NO_I2C2_EV		33
#define IRQ_NO_I2C2_ER		34
#define IRQ_NO_USART1		37
#define IRQ_NO_USART2		38
#define IRQ_NO_USART3		39

#define NVIC_IRQ_PRIO1		1
#define NVIC_IRQ_PRIO15		15


/*
 * Clock enable macros for ADCx peripheral
 */



/*
 * Clock enable macros for TIMx peripheral
 */



/*
 * Clock enable macros for SPIx peripheral
 */





/*
 * Clock enable macros for USARTx peripheral
 */


/*
 * Some generic macros
 */

#define ENABLE 			 1
#define DISABLE 		 0
#define SET				 ENABLE
#define RESET			 DISABLE
#define GPIO_PIN_SET	 SET
#define GPIO_PIN_RESET	 RESET
#define FLAG_SET		 SET
#define FLAG_RESET		 RESET

/***************************************************************************************
 *Bit position definition of SPI Peripheral
 ***************************************************************************************
 */

/*
 * SPI CR1
 */
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15


/*
 * Bit position of SPI CR2
 */
#define SPI_CR2_RXDMAEN		0
#define	SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_RES1		3
#define SPI_CR2_RES2		4
#define SPI_CR2_ERRIE		5
#define	SPI_CR2_RXNEIE		6
#define	SPI_CR2_TXEIE		7
#define SPI_CR2_RESERVED 	8


/*
 * Bit position of SPI SR register
 */
#define SPI_SR_RXNE		0
#define	SPI_SR_TXE		1
#define SPI_SR_CHSIDE	2
#define SPI_SR_UDR		3
#define SPI_SR_CRCERR	4
#define SPI_SR_MODF		5
#define SPI_SR_OVR		6
#define SPI_SR_BSY		7
#define SPI_SR_RESERVED	8


/*
 * Bit position definition of I2C_CR1
 */

#define I2C_CR1_PE			0
#define I2C_CR1_NOSTRETCH	7
#define I2C_CR1_START		8
#define I2C_CR1_STOP		9
#define I2C_CR1_ACK			10
#define I2C_CR1_SWRST		15

/*
 *  Bit position definition of I2C_CR2
 */
#define I2C_CR2_FREQ		0
#define I2C_CR2_ITERREN		8
#define I2C_CR2_ITEVTEN		9
#define I2C_CR2_ITBUFEN		10


/*
 * Bit position definition of I2C_OAR1
 */
#define I2C_OAR1_ADD0		0
#define I2C_OAR1_ADD71		1
#define I2C_OAR1_ADD98		8
#define I2C_OAR1_ADDMODE	15




// Bit position definition of I2C SR1
#define I2C_SR1_SB			0
#define I2C_SR1_ADDR		1
#define I2C_SR1_BTF			2
#define I2C_SR1_ADD10		3
#define I2C_SR1_STOPF		4
#define I2C_SR1_RXNE		6
#define I2C_SR1_TXE			7
#define I2C_SR1_BERR		8
#define I2C_SR1_ARLO		9
#define I2C_SR1_AF			10
#define I2C_SR1_OVR			11
#define I2C_SR1_TIMEOUT 	14


/*
 * Bit position definition of I2C SR2
 */
#define I2C_SR2_MSL			0
#define I2C_SR2_BUSY		1
#define I2C_SR2_TRA			2
#define I2C_SR2_GENCALL		4
#define I2C_SR2_DUALF		7



/*
 * Bit position definition of I2C_CCR
 */
#define I2C_CCR_CCR			0
#define I2C_CCR_DUTY		14
#define I2C_CCR_FS			15

/*
 * Bit position definition of USART_CR1
 */
#define USART_CR1_SBK		0
#define USART_CR1_RWU		1
#define USART_CR1_RE		2
#define USART_CR1_TE		3
#define USART_CR1_IDLEIE	4
#define USART_CR1_RXNEIE	5
#define USART_CR1_TCIE		6
#define USART_CR1_TXEIE		7
#define USART_CR1_PEIE		8
#define USART_CR1_PS		9
#define USART_CR1_PCE		10
#define USART_CR1_WAKE		11
#define USART_CR1_M			12
#define USART_CR1_UE		13

/*
 *Bit position definition of USART_CR2
 */
#define USART_CR2_ADD		0
#define USART_CR2_LBDL		5
#define USART_CR2_LBDIE		6
#define USART_CR2_LBCL		8
#define USART_CR2_CPHA		9
#define USART_CR2_CPOL		10
#define USART_CR2_CLKEN		11
#define USART_CR2_STOP		12
#define USART_CR2_LINEN		14


/*
 * Bit position definition of USART_CR3
 */
#define USART_CR3_EIE		0
#define USART_CR3_IREN		1
#define USART_CR3_IRLP		2
#define USART_CR3_HDSEL		3
#define USART_CR3_NACK		4
#define USART_CR3_SCEN		5
#define USART_CR3_DMAR		6
#define USART_CR3_DMAT		7
#define USART_CR3_RTSE		8
#define USART_CR3_CTSE		9
#define USART_CR3_CTSIE		10


/*
 * Bit position definition of USART_SR
 */
#define USART_SR_PE		0
#define USART_SR_FE		1
#define USART_SR_NE		2
#define USART_SR_ORE	3
#define USART_SR_IDLE	4
#define USART_SR_RXNE	5
#define USART_SR_TC		6
#define USART_SR_TXE	7
#define USART_SR_LBD	8
#define USART_SR_CTS	9





#include"stm32f103xx_gpio_driver.h"
#include"stm32f103xx_spi_driver.h"
#include"stm32f103xx_i2c_driver.h"
#include"stm32f103xx_usart_driver.h"
#include"stm32f103xx_rcc_driver.h"


#endif /* INC_STM32F103XX_H_ */
