/*
 * stm32f103xx_spi_driver.c
 *
 *  Created on: 01-May-2024
 *      Author: shoai
 */


#include"stm32f103xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);
void SPI_delay(void)
{
	for(uint32_t i=0; i<500/2; i++);
}

/*
 * SPI Peripheral clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi )
{
	if(EnorDi== ENABLE)
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_EN();
			}
			else if(pSPIx == SPI2)
			{
				SPI2_PCLK_EN();
			}else if(pSPIx == SPI3)
			{
				SPI3_PCLK_EN();
			}
		}
	else
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_DN();
			}
			else if(pSPIx == SPI2)
			{
				SPI2_PCLK_DN();
			}else if(pSPIx == SPI3)
			{
				SPI3_PCLK_DN();
			}
		}
}



/*
 * Init and DeInit
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{

	//Enable Clock for SPIx peripheral
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE );

	//Let configure SPI_CR1 register

	uint32_t tempreg=0;

	//1.Configure device mode
	tempreg |= pSPIHandle->SPIconfig.SPI_DeviceMode <<SPI_CR1_MSTR;


	//2.Configure the BusConfig

	if (pSPIHandle->SPIconfig.SPI_BusConfig==BUS_CONFIG_FD)
	{
		//Bidi mode should be Cleared
		tempreg &=~ (1<<SPI_CR1_BIDIMODE);

	}
	else if (pSPIHandle->SPIconfig.SPI_BusConfig==BUS_CONFIG_HD)
	{
		//Bidi mode should be Set
		tempreg |= 1<<SPI_CR1_BIDIMODE;
	}
	else if (pSPIHandle->SPIconfig.SPI_BusConfig==BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//Bidi mode should be Cleared
		tempreg &=~ (1<<SPI_CR1_BIDIMODE);

		//RXONLY bit should be Set
		tempreg |= 1<<SPI_CR1_RXONLY;
	}

	//3. Configure SPI serial clock speed
	tempreg |= pSPIHandle->SPIconfig.SPI_SclkSpeed <<SPI_CR1_BR;

	//4. Configure the DFF
	tempreg |= pSPIHandle->SPIconfig.SPI_DFF <<SPI_CR1_DFF	;

	//5. Configure the CPOL
	tempreg |= pSPIHandle->SPIconfig.SPI_CPOL <<SPI_CR1_CPOL;

	//6. Configure the CPHA
	tempreg |= pSPIHandle->SPIconfig.SPI_CPHA <<SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 = tempreg;

	if(pSPIHandle->SPIconfig.SPI_SSM==ENABLE)
		{
			pSPIHandle->pSPIx->CR1|= 1<< SPI_CR1_SSM;

		}
	else
	{
		pSPIHandle->pSPIx->CR1 &=~ (1<< SPI_CR1_SSM);
	}
}






void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_PCLK_DN();
	}
	else if(pSPIx == SPI2)
	{
		SPI2_PCLK_DN();
	}else if(pSPIx == SPI3)
	{
		SPI3_PCLK_DN();
	}
}

/*
 * Date send and data receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len>0)
	{
		//Wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET);

		//2.Check the DFF bit in CR1
		if (pSPIx->CR1 & (1<< SPI_CR1_DFF))
		{
			//16 bit data format
			pSPIx->DR |= *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			//8 bit data format
			//SPI_delay();
			pSPIx->DR |= *pTxBuffer;
			Len--;
			pTxBuffer++;


		}
	}
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi==ENABLE)
	{
		pSPIx->CR1 |= 1<< SPI_CR1_SPE;

	}
	else
	{
		pSPIx->CR1 &=~ (1<< SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi==ENABLE)
	{
		pSPIx->CR1 |= 1<< SPI_CR1_SSI;

	}
	else
	{
		pSPIx->CR1 &=~ (1<< SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi==ENABLE)
	{
		pSPIx->CR2 |= 1<< SPI_CR2_SSOE;

	}
	else
	{
		pSPIx->CR2 &=~ (1<< SPI_CR2_SSOE);
	}
}
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint8_t FlagName)
{
	if (pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	//
	while (Len>0)
	{
		//1.Wait until RXNE bit is zero
		while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG)== FLAG_RESET);

		//2. Check DFF bit in CR1
		if (pSPIx->CR1 & (1<<SPI_CR1_DFF))
		{
			//16bit data format
			//Load 16bit data from DR to RxBuffer address
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			(uint16_t*)pRxBuffer++;
			Len--;
			Len--;
		}
		else
		{
			//8bit data format
			//Load 8bit data from DR to RxBuffer address
			*pRxBuffer = pSPIx->DR;
			pRxBuffer++;
			Len--;
		}

	}
}

/*
 * IRQ configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
		{
			if (IRQNumber <=31)
			{
				//program ISER0
				*NVIC_ISER0 |= 1<< IRQNumber;
			}
			else if (IRQNumber>31 && IRQNumber<64)
			{
				//Program ISER1
				*NVIC_ISER1 |= 1<< (IRQNumber % 32);
			}
			else if (IRQNumber >=64 && IRQNumber<96)
			{
				//Program ISER2
				*NVIC_ISER2 |= 1<< (IRQNumber % 64);
			}
		}
		else
		{
			if (IRQNumber <=31)
			{
				//Program ICER0
				*NVIC_ICER0 |= 1<< IRQNumber;
			}
			else if (IRQNumber>31 && IRQNumber<64)
			{
				//Program ICER1
				*NVIC_ICER1 |= 1<< (IRQNumber % 32);
			}
			else if (IRQNumber >=64 && IRQNumber<96)
			{
				//Program ICER2
				*NVIC_ICER2 |= 1<< (IRQNumber % 64);
			}

		}
}
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{

	//1. First lets find out ipr register
		uint8_t iprx= IRQNumber /4;
		uint8_t iprx_section = IRQNumber %4;

		uint8_t shift_amount = (iprx_section * 8) + ( 8- NO_PR_BITS_IMPLEMENTED);
		*(NVIC_PR_BASE_ADDR + (iprx*4) ) |= (IRQPriority << shift_amount);
}




uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state= pSPIHandle->TxState;

	if (state != SPI_BUSY_IN_TX)
	{

		//1. Save Tx buffer address Length information in some global variable
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		//2. Mark the SPI state busy in transmission so that
		// no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= 1 << SPI_CR2_TXEIE;

		//4. Data transmission will be handled by the ISR code (Will implement later)
	}

	return state;
}


uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{

	uint8_t state=pSPIHandle->TxState;

	if (state!= SPI_BUSY_IN_RX)
	{
	//1. Save Rx buffer address Length information in some global variable
	pSPIHandle->pRxBuffer = pRxBuffer;
	pSPIHandle->RxLen = Len;

	//2. Mark the SPI state busy in Reception so that
	// no other code can take over same SPI peripheral until reception is over
	pSPIHandle->RxState = SPI_BUSY_IN_RX;

	//3. Enable the RXNEIE bit to get interrupt whenever RXNE flag is set in SR
	pSPIHandle->pSPIx->CR2 |= 1 << SPI_CR2_RXNEIE;

	//4. Data Reception will be handled by the ISR code (Will implement later)
	}

	return state;
}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{

	uint8_t temp1,temp2;

	temp1= pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2= pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);
	if (temp1 && temp2)
	{
		//handle txe
		spi_txe_interrupt_handle(pSPIHandle);
	}

	temp1= pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2= pSPIHandle->pSPIx->CR2 & (1<< SPI_CR2_RXNEIE);

	if (temp1 && temp2)
	{
		//handle rxne
		spi_rxne_interrupt_handle(pSPIHandle);

	}

	temp1= pSPIHandle->pSPIx->SR & (1<< SPI_SR_OVR);
	temp2= pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if (temp1 && temp2)
	{
		//handle ovr error
		spi_ovr_err_interrupt_handle(pSPIHandle);
	}

}


static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//1.Check the DFF bit in CR1
	if (pSPIHandle->pSPIx->CR1 & (1<< SPI_CR1_DFF))
	{
		//16 bit data format
		pSPIHandle->pSPIx->DR |= *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}
	else
	{
		//8 bit data format
		pSPIHandle->pSPIx->DR |= *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if (! pSPIHandle->TxLen )
	{
		// Tx Len is zero, so close the spi transmission and inform the application that
		// Tx is over
		SPI_CloseTransmission(pSPIHandle);
		SPI_Application_EventCallBack(pSPIHandle,SPI_EVENT_TX_CMPLT);

	}

}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//1. Check DFF bit in CR1
	if (pSPIHandle->pSPIx->CR1 & (1<<SPI_CR1_DFF))
	{
		//16bit data format
		//Load 16bit data from DR to RxBuffer address
		*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		(uint16_t*)pSPIHandle->pRxBuffer++;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
	}
	else
	{
		//8bit data format
		//Load 8bit data from DR to RxBuffer address
		*pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->RxLen--;
	}

	if (! pSPIHandle->RxLen)
	{
		SPI_CloseReception(pSPIHandle);
		SPI_Application_EventCallBack(pSPIHandle,SPI_EVENT_RX_CMPLT);

	}
}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//1.Clear the OVR flag
	if (pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp=pSPIHandle->pSPIx->DR;
		temp=pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//2. Inform the application
	SPI_Application_EventCallBack(pSPIHandle,SPI_EVENT_OVR_ERR);
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp=pSPIx->DR;
	temp=pSPIx->SR;
	(void)temp;
}


void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~ (1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer= NULL;
	pSPIHandle->TxLen =0;
	pSPIHandle->TxState = SPI_READY;


}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &=~ (1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}

__weak void SPI_Application_EventCallBack(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	//This is a weak implementation. The user Application may override this function
}
