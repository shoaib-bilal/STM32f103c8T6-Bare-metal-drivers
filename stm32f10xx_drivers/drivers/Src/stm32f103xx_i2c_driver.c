/*
 * stm32f103xx_i2c_driver.c
 *
 *  Created on: 01-Jun-2024
 *      Author: shoai
 */
#include"stm32f103xx_i2c_driver.h"


static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);

void ComDelay()
{
	for(uint32_t i=0; i<5000/2; i++);
}

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= 1<<I2C_CR1_START;
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr= SlaveAddr<<1;
	SlaveAddr &= ~(1<<0);   //Clearing 1st bit means it will write data next
							//Slave address is slave address + r/w bit=0
	pI2Cx->DR |= SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr= SlaveAddr<<1;
	SlaveAddr |= 1<<0;   //setting 1st bit means it will Read data next
							//Slave address is slave address + r/w bit=1
	pI2Cx->DR |= SlaveAddr;
}
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummyread;
	//Check device mode
	if (pI2CHandle->pI2Cx->SR2 &= (1<< I2C_SR2_MSL ))
	{
		//Device is in master mode
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX )
		{
			if (pI2CHandle->RxSize ==1)
			{
				//1. first disable acking
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
				dummyread= pI2CHandle->pI2Cx->SR1;
				dummyread= pI2CHandle->pI2Cx->SR2;
				(void)dummyread;
			}
		}
		else
		{
			//Clear ADDR flag
			dummyread= pI2CHandle->pI2Cx->SR1;
			dummyread= pI2CHandle->pI2Cx->SR2;
			(void)dummyread;

		}
	}
	else
	{
		//Clear ADDR flag
		dummyread= pI2CHandle->pI2Cx->SR1;
		dummyread= pI2CHandle->pI2Cx->SR2;
		(void)dummyread;

	}


}


void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= 1<< I2C_CR1_STOP;

}

void I2C_SlaveEnableDisableCallBackEvents(I2C_RegDef_t *pI2Cx,uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{

		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}
	else
	{
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITERREN);

	}
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx,uint8_t FlagName)
{
	if (pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi )
{
	if (EnorDi == ENABLE)
	{
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
	}
	else
	{
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_DN();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_DN();
		}
	}
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{

	if (EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= 1<< I2C_CR1_PE;
	}
	else
	{
		pI2Cx->CR1 &=~ 1<< I2C_CR1_PE;
	}
}


/*
 * Init and DeInit
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg=0;

	//Enable Clock for I2Cx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	//ACK control
	tempreg= (pI2CHandle->I2C_Config.I2C_ACKControl ) ;
	pI2CHandle->pI2Cx->CR1 |= tempreg <<10;

	//Configure the Freq field of CR2 register
	tempreg=0;
	tempreg = RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	//Program the device own address
	tempreg =0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DEVICEAddress <<1;
	tempreg |= 1<<14;
	pI2CHandle->pI2Cx->OAR1 |= tempreg;

	//CCR calculation
	uint16_t ccr_value =0;
	tempreg=0;

	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//Mode is standard mode
		tempreg &= ~ (1<<15);

		ccr_value = RCC_GetPCLK1Value() / ( 2* pI2CHandle->I2C_Config.I2C_SCLSpeed);

		tempreg |= ccr_value & 0xFFF;

	}
	else
	{
		//Mode is Fast mode
		tempreg |= 1<<15;
		tempreg |= pI2CHandle->I2C_Config.I2C_FMDUTYCycle <<14;
		if (pI2CHandle->I2C_Config.I2C_FMDUTYCycle == I2C_FM_DUTY_2)
		{
			ccr_value = RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		else
		{
			ccr_value = RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		tempreg |= ccr_value & 0xFFF;


	}
	pI2CHandle->pI2Cx->CCR |= tempreg;

	tempreg=0;

	//Trise configuration

	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//Mode is standard mode

		tempreg=  (RCC_GetPCLK1Value() / 1000000U) +1 ;
		//tempreg=9;

	}
	else
	{
		tempreg= ((RCC_GetPCLK1Value() * 300)/ 1000000000) +1 ;
	}

	pI2CHandle->pI2Cx->TRISE |= (tempreg & 0x3F);

}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	//1.Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm start generation is completed by checking the SB flag in SR1
	// Note: Until SB is cleared SCL will be stretched (Pulled to Low)
	while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//I2C_ClearSBFlag(pI2CHandle->pI2Cx);

	//3. Send the address of the Slave with r/w bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	//4.Confirm that address phase is completed by checking the addr flag in SR1
	while (!I2C_GetFlagStatus (pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//5. Clear addr Flag by reading SR1 followed by SR2
	I2C_ClearADDRFlag(pI2CHandle);

	//6. Send the data until Len becomes 0
	while(Len>0)
	{
		while(! I2C_GetFlagStatus (pI2CHandle->pI2Cx, I2C_FLAG_TXE));

		pI2CHandle->pI2Cx->DR |= *pTxBuffer;
		pTxBuffer++;
		Len--;
		//ComDelay();
	}

	//7. When Len becomes zero, wait for TXE=1 and BTF=1 before generating the stop condition
	// Note: TXE=1 and BTF=1 means both SR and DR are empty and next transmission should begin
	// When BTF=1 SCL will be stretched (Pulled to Low)
	while(! I2C_GetFlagStatus (pI2CHandle->pI2Cx, I2C_FLAG_TXE));

	while(! I2C_GetFlagStatus (pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	//8. Generate STOP condition and master need not to wait for the completion of stop condition
	// Note: Generating stop condition clears the BTF
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);


}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	//1.Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm start generation is completed by checking the SB flag in SR1
	// Note: Until SB is cleared SCL will be stretched (Pulled to Low)
	while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. Send the address of the Slave with r/w bit set to R(1) (total 8 bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	//4.Confirm that address phase is completed by checking the addr flag in SR1
	while (!I2C_GetFlagStatus (pI2CHandle->pI2Cx, I2C_FLAG_ADDR));



	//4.Procedure to read only 1 byte from Slave
	if (Len ==1)
	{
		//Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);



		//Clear ADDR flag
		// Clear addr Flag by reading SR1 followed by SR2
		I2C_ClearADDRFlag(pI2CHandle);

		//Wait until RxNE becomes 1
		while (!I2C_GetFlagStatus (pI2CHandle->pI2Cx, I2C_SR1_RXNE));

		//Generate stop condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);


		//Read data into Buffer
		*pTxBuffer = pI2CHandle->pI2Cx->DR;

		return;
	}

	//Procedure to read more than 1 byte from Slave
	if (Len>1)
	{
		//Clear ADDR Flag
		I2C_ClearADDRFlag(pI2CHandle);

		for (uint32_t i=Len; i>0 ;i--)
		{
			//Wait until RxNE becomes 1
			while (!I2C_GetFlagStatus (pI2CHandle->pI2Cx, I2C_SR1_RXNE));

			if (i==2)
			{
				//Clear the ACK bit
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				//Generate stop condition
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			//Read data into Buffer
			*pTxBuffer = pI2CHandle->pI2Cx->DR;

			//Increment buffer address
			pTxBuffer++;

		}
	}
	//Re-enable Acking
	if (pI2CHandle->I2C_Config.I2C_ACKControl ==I2C_ACK_ENABLE )
		{
			I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
		}
}




void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{

	if (pI2Cx == I2C1)
	{
		I2C1_PCLK_DN();
	}
	else if (pI2Cx == I2C2)
	{
		I2C2_PCLK_DN();
	}
}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == I2C_ACK_ENABLE)
	{
		pI2Cx->CR1 |= 1<<I2C_CR1_ACK;
	}
	else
	{
		pI2Cx->CR1 &= ~ (1<<I2C_CR1_ACK);
	}

}



uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

		if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
		{
			pI2CHandle->pTxBuffer = pTxBuffer ;
			pI2CHandle->TxLen = Len;
			pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
			pI2CHandle->DevAddr = SlaveAddr;
			pI2CHandle->Sr = Sr;

			//Implement code to Generate START Condition
			I2C_GenerateStartCondition(pI2CHandle->pI2Cx);


			//Implement the code to enable ITBUFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

			//Implement the code to enable ITEVFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);


			//Implement the code to enable ITERREN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);


		}

		return busystate;

}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer ;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState =I2C_BUSY_IN_RX ;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);


		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);


		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);


		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;


}

void I2C_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi)
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

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{

	//1. First lets find out ipr register
		uint8_t iprx= IRQNumber /4;
		uint8_t iprx_section = IRQNumber %4;

		uint8_t shift_amount = (iprx_section * 8) + ( 8- NO_PR_BITS_IMPLEMENTED);
		*(NVIC_PR_BASE_ADDR + (iprx*4) ) |= (IRQPriority << shift_amount);

}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{

	//Implement the code to Disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);


	//Implement the code to Disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen= 0;
	pI2CHandle->RxSize= 0;

	if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
	I2C_ManageAcking(pI2CHandle->pI2Cx,ENABLE);
	}
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to Disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);


	//Implement the code to Disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer= NULL;
	pI2CHandle->TxLen=0;




}

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t *data)
{
	pI2Cx->DR = *data;

}
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
	return pI2Cx->DR;
}
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if (pI2CHandle->TxLen >0)
	{
		pI2CHandle->pI2Cx->DR |= *(pI2CHandle->pTxBuffer);
		pI2CHandle->pTxBuffer++;
		pI2CHandle->TxLen--;
	}

}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{

	if (pI2CHandle->RxSize ==1)
	{
		*(pI2CHandle->pRxBuffer)= pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxSize >1)
	{
		if (pI2CHandle->RxLen ==2)
		{
			I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
		}

	*(pI2CHandle->pRxBuffer)= pI2CHandle->pI2Cx->DR;
	pI2CHandle->RxLen--;
	pI2CHandle->pRxBuffer++;

	}

	if (pI2CHandle->RxLen ==0)
	{
		//Generate stop condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		I2C_CloseReceiveData(pI2CHandle);

		I2C_Application_EventCallBack(pI2CHandle,I2C_EV_RX_CMPLT);

	}

}




void I2C_EV_IRQ_Handling(I2C_Handle_t *pI2CHandle)
{
	//Interrupt handling for both master and slave mode of a device
	uint32_t temp1,temp2,temp3;

	temp1= pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2= pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

	temp3= pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);

	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode
	if (temp1 && temp3)
	{
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
		I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
		else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
		I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	temp3= pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	//2. Handle For interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//		 When Slave mode   : Address matched with own address
	if (temp1 && temp3)
	{
		I2C_ClearADDRFlag(pI2CHandle);
	}

	temp3= pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event

	if (temp1 && temp3)
	{
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//Make sure TXE is also set
			if (pI2CHandle->pI2Cx->SR1 & (1<< I2C_SR1_TXE) )
			{
				//BTF=1 TXE=1
				//1. Generate stop condition
				if (pI2CHandle->TxLen ==0)
				{
					if (pI2CHandle->Sr == I2C_DISABLE_SR)
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);


					//2. Reset all member elements of handle structure
					I2C_CloseSendData(pI2CHandle);


					//3.Notify the application about transmission complition
					I2C_Application_EventCallBack(pI2CHandle,I2C_EV_TX_CMPLT);
				}
			}

		}
		else if ((pI2CHandle->TxRxState == I2C_BUSY_IN_RX))
		{
			;
		}

	}
	temp3= pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set


	if (temp1 && temp3)
	{
		//STOP Flag is set
		//Clear STOPF (i.e. 1) Read SR1 and 2) write to CR1)

		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		//Notify the application that stop is detected
		I2C_Application_EventCallBack(pI2CHandle,I2C_EV_STOP);
	}


	temp3= pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
	//5. Handle For interrupt generated by TXE event
	if (temp1 && temp2 && temp3)
	{
		//Check for device mode
		if (pI2CHandle->pI2Cx->SR2 & (1<< I2C_SR2_MSL) )
		{
			//TXE flag is set
			// We have to do transmission
			if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}
		else
		{
			//SLave
			if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
			{
			I2C_Application_EventCallBack(pI2CHandle,I2C_EV_DATA_REQ);
			}

		}
	}

	temp3= pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
	//6. Handle For interrupt generated by RXNE event
	if (temp1 && temp2 && temp3)
	{
		if (pI2CHandle->pI2Cx->SR2 &= (1<< I2C_SR2_MSL ))
			{
				//RXNE flag is set
				if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX )
				{
					I2C_MasterHandleRXNEInterrupt(pI2CHandle);
				}
			}
		else
		{
			//Slave
			//Make sure slave is really in receiver mode
			if (!(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)))
			{
			I2C_Application_EventCallBack(pI2CHandle,I2C_EV_DATA_RCV);
			}
		}
	}

}
/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Complete the code also define these macros in the driver
						header file
						#define I2C_ERROR_BERR  3
						#define I2C_ERROR_ARLO  4
						#define I2C_ERROR_AF    5
						#define I2C_ERROR_OVR   6
						#define I2C_ERROR_TIMEOUT 7

 */

void I2C_ER_IRQ_Handling(I2C_Handle_t *pI2CHandle)
{



	uint32_t temp1,temp2;

	//Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_Application_EventCallBack(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_Application_EventCallBack(pI2CHandle,I2C_ERROR_ARLO);

	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

		//Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_Application_EventCallBack(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

		//Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_Application_EventCallBack(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

		//Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_Application_EventCallBack(pI2CHandle,I2C_ERROR_TIMEOUT);
	}

}

__weak void I2C_Application_EventCallBack(I2C_Handle_t *pI2CHandle,uint8_t AppEv)
{
	//This is a weak implementation. The user Application may override this function
}


