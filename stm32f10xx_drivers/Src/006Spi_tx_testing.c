/*
 * 006Spi_tx.c
 *
 *  Created on: 05-May-2024
 *      Author: shoai
 */

/*
 * PA4->NSS1
 * PA5->SCK1
 * PA6->MISO1
 * PA7->MOSI1
 *
 */

#include"stm32f103xx_spi_driver.h"
#include<string.h>

void delay()
{
	for(uint32_t i=0; i<500000/2; i++);
}

void SPI1_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;
	memset(&SPIPins,0,sizeof(SPIPins));

	SPIPins.pGPIOx= GPIOA;
	SPIPins.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_OUT_2Mhz;
	SPIPins.GPIO_PinConfig.GPIO_PinOpType=GPIO_OP_AL_PUSHPULL;

	//SCK1
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_5;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_7;
	GPIO_Init(&SPIPins);

	//MISO
	//SPIPins->GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_6;
	//GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_4;
	GPIO_Init(&SPIPins);

}


void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;
	memset(&SPIPins,0,sizeof(SPIPins));

	SPIPins.pGPIOx= GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_OUT_2Mhz;
	SPIPins.GPIO_PinConfig.GPIO_PinOpType=GPIO_OP_AL_PUSHPULL;

	//SCK1
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	//SPIPins->GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_6;
	//GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

}
void SPI1_Inits(void)
{

	SPI_Handle_t SPI1Handle;
	SPI1Handle.pSPIx= SPI2;
	SPI1Handle.SPIconfig.SPI_BusConfig = BUS_CONFIG_FD;
	SPI1Handle.SPIconfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI1Handle.SPIconfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV256;
	SPI1Handle.SPIconfig.SPI_DFF = SPI_DFF_8BITS;
	SPI1Handle.SPIconfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI1Handle.SPIconfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI1Handle.SPIconfig.SPI_SSM = SPI_SSM_DN;

	SPI_Init(&SPI1Handle);
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

int main()
{
	GPIO_ButtonInit();

	char user_data[]="Hello world";

	//This function is used to initialize GPIO pins to behave as SPI pins
	//SPI1_GPIOInits();

	SPI2_GPIOInits();

	//This function is used to initialize the SPI2 peripheral parameters
	SPI1_Inits();

	//Set the SSI bit
	//SPI_SSIConfig(SPI1,ENABLE);

	SPI_SSOEConfig(SPI2, ENABLE);
	while(1)
	{

		while (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_1));
		delay();

		//Enable SPI1 Peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		//First send length information
		//uint8_t dataLen = strlen(user_data);

		//SPI_SendData(SPI1, &dataLen,1);

		//This function is used to send data over SPI
		SPI_SendData(SPI2, (uint8_t*)user_data,strlen(user_data));

		while(SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG));
		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}
