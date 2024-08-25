/*
 * stm32f103xx_spi_driver.h
 *
 *  Created on: 01-May-2024
 *      Author: shoai
 */

#ifndef INC_STM32F103XX_SPI_DRIVER_H_
#define INC_STM32F103XX_SPI_DRIVER_H_

#include "stm32f103xx.h"


/*
 * Configuration structure of SPIx peripheral
 */

typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

}SPI_Config_t;

typedef struct
{
	SPI_RegDef_t *pSPIx;    //This holds the base address of SPIx
	SPI_Config_t SPIconfig;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxState;
	uint8_t RxState;

}SPI_Handle_t;


/*
 * Device mode
 */
#define SPI_DEVICE_MODE_MASTER		1
#define SPI_DEVICE_MODE_SLAVE		0

/*
 * @Bus config
 */
#define BUS_CONFIG_FD				1
#define BUS_CONFIG_HD				2
//#define BUS_CONFIG_SIMPLEX_TXONLY	3  Skipped- can be accomplished by disconnecting RX(MISO pin)
#define BUS_CONFIG_SIMPLEX_RXONLY	3

/*
 * @SPI SCLK Speed
 */
#define SPI_SCLK_SPEED_DIV2			0
#define SPI_SCLK_SPEED_DIV4			1
#define SPI_SCLK_SPEED_DIV8			2
#define SPI_SCLK_SPEED_DIV16		3
#define SPI_SCLK_SPEED_DIV32		4
#define SPI_SCLK_SPEED_DIV64		5
#define SPI_SCLK_SPEED_DIV128		6
#define SPI_SCLK_SPEED_DIV256		7


/*
 * @SPI DFF
 */
#define SPI_DFF_8BITS	0
#define SPI_DFF_16BITS	1

/*
 *@CPOL
 */
#define SPI_CPOL_HIGH	1
#define SPI_CPOL_LOW	0

/*
 *@CPHA
 */
#define SPI_CPHA_HIGH	1
#define SPI_CPHA_LOW	0

/*
 * @SSM
 */
#define SPI_SSM_EN		1
#define SPI_SSM_DN		0

/*
 *@SPI related Flag definitions
 */
#define SPI_TXE_FLAG	1<<SPI_SR_TXE
#define SPI_RXNE_FLAG	1<<SPI_SR_RXNE
#define SPI_BUSY_FLAG	1<<SPI_SR_BSY


/*
 *SPI Application states
 */
#define SPI_READY		0
#define SPI_BUSY_IN_RX	1
#define SPI_BUSY_IN_TX	2

/*
 * Possible SPI application events
 */
#define SPI_EVENT_TX_CMPLT		1
#define SPI_EVENT_RX_CMPLT		2
#define SPI_EVENT_OVR_ERR		3
#define SPI_EVENT_CRC			4

/***********************************************************************************
*								API Supported by this driver
*		For more information about the APIs check the function definitions
************************************************************************************
*/


/*
 * Peripheral clock setup
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi );

/*
 * Init and DeInit
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * SPI Peripheral Enable or Disable
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/*
 * SPI SSI bit config
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
/*
 * Date send and data receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);


uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);
/*
 * IRQ configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint8_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Application call back
 */
void SPI_Application_EventCallBack(SPI_Handle_t *pSPIHandle,uint8_t AppEv);

#endif /* INC_STM32F103XX_SPI_DRIVER_H_ */
