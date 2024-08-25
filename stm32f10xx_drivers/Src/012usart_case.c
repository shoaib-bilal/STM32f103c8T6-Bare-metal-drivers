/*
 * 012usart_case.c
 *
 *  Created on: 24-Jul-2024
 *      Author: shoai
 */


#include<string.h>
#include<stdio.h>
#include"stm32f103xx.h"
#include"stm32f103xx_usart_driver.h"


char *msg[3]= {"HiHiHi","How are you\n","What do you do\n"};

USART_Handle_t USART3Handle;



uint8_t rx_cmplt= RESET;

char rcv_buff[1024];

void delay()
{
	for(uint32_t i=0; i<500000/2; i++);
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
	USART3Handle.USART_Config.USART_Mode= USART_MODE_TXRX;
	USART3Handle.USART_Config.USART_NoOfStopBits= USART_STOPBITS_1;
	USART3Handle.USART_Config.USART_WordLength= USART_WORDLEN_8BITS;
	USART3Handle.USART_Config.USART_ParityControl= USART_PARITY_DISABLE;




	USART_Init(&USART3Handle);
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
	uint32_t cnt= 0;

	//Button Init
	GPIO_ButtonInit();

	//This function is used to initialize GPIO pins to behave as USART3 pins
	USART3_GPIOInits();

	//This function is used to initialize the USART3 peripheral parameters
	USART3_Inits();

	USART_IRQInterruptConfig(IRQ_NO_USART3, ENABLE);


	//Enable USART3 Peripheral
	USART_PeripheralControl(USART3, ENABLE);





	while(1)
	{


		//Wait till button is pressed
		//while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_1));

		//delay to avoid button de-bouncing related issue
		//delay();

		cnt=cnt % 3;

		while(USART_ReceiveDataIT(&USART3Handle,(uint8_t*)rcv_buff, strlen(msg[cnt])) != USART_READY );

		USART_SendData(&USART3Handle, (uint8_t*)msg[cnt], strlen(msg[cnt]) );

		//USART_SendDataIT(&USART3Handle, (uint8_t*)rcv_buff, strlen(msg[cnt]) );




		while(rx_cmplt != SET);

		rcv_buff[strlen(msg[cnt])+1]= '\0';

		rx_cmplt = RESET;


		//USART_SendData(&USART3Handle, (uint8_t*)rcv_buff, strlen(rcv_buff) );
		cnt++;



	}
	return 0;
}


void USART3_IRQHandler(void)
{
	USART_IRQHandling(&USART3Handle);
}
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv)
{
	if (AppEv == USART_EVENT_RX_CMPLT)
	{
		rx_cmplt= SET;
	}
	else if (AppEv == USART_EVENT_TX_CMPLT)
   {
	   ;
   }

}
