/*
 * 011usart_tx_testing.c
 *
 *  Created on: 19-Jul-2024
 *      Author: shoai
 */


#include<string.h>
#include"stm32f103xx.h"
#include"stm32f103xx_usart_driver.h"

char rcv_buff[1024];
USART_Handle_t USART3Handle;
uint8_t changecase(uint8_t ch);

void delay()
{
	for(uint32_t i=0; i<500000/2; i++);
}

char msg[1024]= "We are testing UART tx\n\r";

char msg1[1024]="We are sending data from arduino UART\n\r";



//uint8_t some_data[]= {0xBC, 0xBC,0x55,0xB2,0xB3};

//uint8_t some_data[]= {0b10101010, 0b10101010,0b10101010,0b10101010,0b10101010};




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



	//Enable USART3 Peripheral
	USART_PeripheralControl(USART3Handle.pUSARTx, ENABLE);

	//Send some data

	//USART_SendData(&USART3Handle, (uint8_t*)msg, strlen(msg));



	//rcv_buff[strlen(msg)+1]= '\0';



	while(1)
	{
		//while(USART_ReceiveDataIT(&USART3Handle, (uint8_t*)rcv_buff,  strlen(msg1)) != USART_READY);
		//USART_SendData(&USART3Handle, (uint8_t*)msg, strlen(msg));
		//USART_SendData(&USART3Handle, (uint8_t*)rcv_buff, strlen(msg1));
		USART_ReceiveData(&USART3Handle, (uint8_t*)rcv_buff, strlen(msg));
		USART_SendData(&USART3Handle, (uint8_t*)rcv_buff, strlen(msg));
	}
}

uint8_t changecase(uint8_t ch)
{
  if (ch >= 'A' && ch <= 'Z')
  {
  ch = ch + 32;
  }
  else if (ch >= 'a' && ch <= 'z')
  {
	  ch = ch - 32;
  }

  return ch;
}

