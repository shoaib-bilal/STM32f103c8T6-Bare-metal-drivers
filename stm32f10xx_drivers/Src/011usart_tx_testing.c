/*
 * 011usart_tx_testing.c
 *
 *  Created on: 19-Jul-2024
 *      Author: shoai
 */


#include<string.h>
#include"stm32f103xx.h"
#include"stm32f103xx_usart_driver.h"

uint8_t rcv_buff[1024];
USART_Handle_t USART3Handle;

uint8_t subtract(uint8_t rcvbuff);

void delay()
{
	for(uint32_t i=0; i<50000/2; i++);
}

//char msg[1024]= "We are testing UART tx\n\r";

uint8_t msg[1024]= {0x55,0x55,0x55};


char msg1[1024]="We are sending data from arduino UART\n\r";



//uint8_t some_data[]= {0xBC, 0xBC,0x55,0xB2,0xB3};

//uint8_t some_data[]= {0b10101010, 0b10101010,0b10101010,0b10101010,0b10101010};


void GPIO_LedInit(void)
{
	    GPIO_Handle_t GpioLed;
		memset(&GpioLed,0, sizeof(GpioLed));

		GpioLed.pGPIOx = GPIOC;
		GpioLed.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_13;
		GpioLed.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_OUT_50Mhz;
		GpioLed.GPIO_PinConfig.GPIO_PinOpType= GPIO_OP_TYPE_PUSHPULL;

		GPIO_Init(&GpioLed);
}

void USART3_GPIOInits(void)
{
	GPIO_Handle_t USARTPins;
	memset(&USARTPins,0,sizeof(USARTPins));


	USARTPins.pGPIOx = GPIOB;
	USARTPins.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_OUT_50Mhz;
	USARTPins.GPIO_PinConfig.GPIO_PinOpType= GPIO_OP_AL_PUSHPULL;




	//Tx
	USARTPins.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_10;
	GPIO_Init(&USARTPins);


	//Rx
	USARTPins.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_11;
	GPIO_Init(&USARTPins);



}
void USART3_Inits(void)
{

	memset(&USART3Handle,0,sizeof(USART3Handle));
	USART3Handle.pUSARTx = USART3;
	USART3Handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	USART3Handle.USART_Config.USART_HWFlowControl= USART_HW_FLOW_CTRL_NONE;
	USART3Handle.USART_Config.USART_Mode= USART_MODE_ONLY_TX;
	USART3Handle.USART_Config.USART_NoOfStopBits= USART_STOPBITS_1;
	USART3Handle.USART_Config.USART_WordLength= USART_WORDLEN_8BITS;
	USART3Handle.USART_Config.USART_ParityControl= USART_PARITY_DISABLE;




	USART_Init(&USART3Handle);
}


int main(void)
{


	//This function is used to initialize GPIO pins to behave as USART3 pins
	USART3_GPIOInits();

	//This function is used to initialize the USART3 peripheral parameters
	USART3_Inits();
	GPIO_LedInit();


	//Enable USART3 Peripheral
	USART_PeripheralControl(USART3Handle.pUSARTx, ENABLE);



	uint32_t i=0;

	while(1)
	{
		//while(USART_ReceiveDataIT(&USART3Handle, (uint8_t*)rcv_buff,  strlen(msg1)) != USART_READY);
		USART_SendData(&USART3Handle, msg, strlen(msg));
		//USART_SendData(&USART3Handle, (uint8_t*)rcv_buff, strlen(msg1));
		USART_ReceiveData(&USART3Handle, rcv_buff, strlen(msg));
		if (rcv_buff==0x55)
		{

			GPIO_ToggleOutputPin(GPIOC,GPIO_PIN_NO_13);
			delay();
		}

		else
		{
		//	GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_NO_13, RESET);
		}

	}
}

uint8_t subtract(uint8_t rcvbuff)
{
	rcvbuff= rcvbuff-5;
	return rcvbuff;
}
