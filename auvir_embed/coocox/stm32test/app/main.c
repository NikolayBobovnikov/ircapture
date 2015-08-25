#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include <stdio.h>

void InitUSART()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    USART_Cmd(USART1, ENABLE);
}
void initLED()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	GPIO_InitTypeDef gpio;

	gpio.GPIO_Pin = GPIO_Pin_8;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &gpio);

	gpio.GPIO_Pin = GPIO_Pin_9;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &gpio);
}

void delay(int num)
{
	unsigned int i;
	for (i = 0; i < num; i++)
	{
		;
	}
}

void print()
{
	setvbuf(stdin, NULL, _IONBF, 0);
	setvbuf(stdout, NULL, _IONBF, 0);
	setvbuf(stderr, NULL, _IONBF, 0);
	printf("Hello, World!!!");
}

int main(void)
{
	initLED();
	InitUSART();

	while(1)
    {
		GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_SET);
	    GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_RESET);
	    delay(1000000);
	    GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_RESET);
	    GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_SET);
	    delay(1000000);

	    print();
    }
}
