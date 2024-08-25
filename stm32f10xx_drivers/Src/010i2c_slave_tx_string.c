/*
 * 010i2c_slave_tx_string.c
 *
 *  Created on: 07-Jul-2024
 *      Author: shoai
 */

/*
 * 009i2c_master_rx_testingIT.c
 *
 *  Created on: 1-July-2024
 *      Author: shoai
 */

/*
 * SCL->PB10
 * SDA->PB11
 *
 */


#include<string.h>
#include"stm32f103xx.h"
#include"stm32f103xx_i2c_driver.h"


#define SLAVE_ADDR 0x68

#define MY_ADDR SLAVE_ADDR

I2C_Handle_t I2C2Handle;



void delay()
{
	for(uint32_t i=0; i<500000/2; i++);
}

uint8_t Txbuff[32]= "STM32 slave mode testing....";



void I2C2_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;
	memset(&I2CPins,0,sizeof(I2CPins));


	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_OUT_50Mhz;
	I2CPins.GPIO_PinConfig.GPIO_PinOpType= GPIO_OP_AL_OPENDRAIN;


	//GPIOB_PCLK_EN();
	//GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_10, SET);
	//GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_11, SET);

	//SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_10;
	GPIO_Init(&I2CPins);


	//SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_11;
	GPIO_Init(&I2CPins);



}
void I2C2_Inits(void)
{

	memset(&I2C2Handle,0,sizeof(I2C2Handle));
	I2C2Handle.pI2Cx = I2C2;
	I2C2Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C2Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C2Handle.I2C_Config.I2C_DEVICEAddress = MY_ADDR;     // no purpose here as STM32 is master
	I2C2Handle.I2C_Config.I2C_FMDUTYCycle = I2C_FM_DUTY_2; // no purpose here as Standard mode



	I2C_Init(&I2C2Handle);
}



int main(void)
{



	//This function is used to initialize GPIO pins to behave as I2C pins
	I2C2_GPIOInits();

	//This function is used to initialize the I2C1 peripheral parameters
	I2C2_Inits();

	I2C_IRQInterruptConfig(IRQ_NO_I2C2_EV,ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C2_ER,ENABLE);

	I2C_SlaveEnableDisableCallBackEvents(I2C2,ENABLE);

	//Enable I2C1 Peripheral
	I2C_PeripheralControl(I2C2, ENABLE);
	I2C_ManageAcking(I2C2,I2C_ACK_ENABLE);

	while(1);

}

void I2C2_EV_IRQHandler(void)
{


	I2C_EV_IRQ_Handling(&I2C2Handle);
}


void I2C2_ER_IRQHandler(void)
{
	I2C_ER_IRQ_Handling(&I2C2Handle);


}


void I2C_Application_EventCallBack(I2C_Handle_t *pI2CHandle,uint8_t AppEv)
{
	static uint8_t commandcode=0;
	static uint8_t Cnt=0;
	if (AppEv == I2C_EV_DATA_REQ)
	{
		//master wants some data. Slave has to send it
		if (commandcode ==0x51)
		{
			//Send the length information to the master
			I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen((char*)Txbuff));
		}
		else if (commandcode ==0x52)
		{
			//Send the content of Tx
			I2C_SlaveSendData(pI2CHandle->pI2Cx, Txbuff[Cnt++]);
		}

	}
	else if (AppEv == I2C_EV_DATA_RCV)
	{
		//Data is waiting for the slave to read. Slave has to read it
		commandcode= I2C_SlaveReceiveData(pI2CHandle->pI2Cx);
	}
	else if (AppEv == I2C_ERROR_AF)
	{
		//This happens only during slave transmission
		//Master has sent NACK. So slave should understand the master doesn't
		//need more data
		commandcode=0xff;
		Cnt=0;


	}

}



