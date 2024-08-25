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

#define MY_ADDR 0x61
#define SLAVE_ADDR 0x68

I2C_Handle_t I2C2Handle;

uint8_t rxComplt= RESET;

void delay()
{
	for(uint32_t i=0; i<500000/2; i++);
}

//uint8_t some_data[]= "We are testing I2C master tx";

//uint8_t some_data[]= {0xBC, 0xBC,0x55,0xB2,0xB3};

uint8_t rcvbuff[32];



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

void GPIO_ButtonInit()
{

	GPIO_Handle_t GpioButton;
	memset(&GpioButton,0,sizeof(GpioButton));
	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_1;
	GpioButton.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinInType= GPIO_IN_TYPE_PULLUP;

	//GPIOA_PCLK_EN();      No need as clock enable is included in driver code
	GPIO_Init(&GpioButton);
}

int main(void)
{

	uint8_t commandcode;
	uint8_t len;
	GPIO_ButtonInit();

	//This function is used to initialize GPIO pins to behave as I2C pins
	I2C2_GPIOInits();

	//This function is used to initialize the I2C1 peripheral parameters
	I2C2_Inits();

	I2C_IRQInterruptConfig(IRQ_NO_I2C2_EV,ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C2_ER,ENABLE);



	//Enable I2C1 Peripheral
	I2C_PeripheralControl(I2C2, ENABLE);
	I2C2Handle.pI2Cx->CR1 |= 1<<10;



	while(1)
	{

		//Wait till button is pressed
		while (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_1));

		//delay to avoid button de-bouncing related issue
		delay();

	//	commandcode=0x51;

	//	while(I2C_MasterSendDataIT(&I2C2Handle,&commandcode,1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);


	//	while(I2C_MasterReceiveDataIT(&I2C2Handle,&len,1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);

		commandcode=0x52;
		while(I2C_MasterSendDataIT(&I2C2Handle,&commandcode,1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);

		while(I2C_MasterReceiveDataIT(&I2C2Handle,rcvbuff,23, SLAVE_ADDR, I2C_DISABLE_SR) != I2C_READY);

		rxComplt= RESET;

		while (rxComplt != SET);

		rcvbuff[len+1]='\0';

		rxComplt= RESET;
	}

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
	if (AppEv == I2C_EV_RX_CMPLT)
	{
		rxComplt = SET;

	}
	else if (AppEv == I2C_ERROR_AF)
	{
		I2C_CloseSendData(pI2CHandle);
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		while(1);
	}

}

