#include "stm32f10x_conf.h"

/* led connected to a gpio pin */
#define LED1_PIN    GPIO_Pin_0
#define LED1_PORT   GPIOB
#define LED2_PIN    GPIO_Pin_3
#define LED2_PORT   GPIOC
#define LED3_PIN    GPIO_Pin_0
#define LED3_PORT   GPIOA
#define LED4_PIN    GPIO_Pin_0
#define LED4_PORT   GPIOE


/* user functions */
void delay(unsigned long count);

int main()
{
    GPIO_InitTypeDef GPIO_InitStructure;



    /* enable clock on GPIOB peripheral */
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOA, ENABLE);                          


    /* set pin output mode */
    GPIO_InitStructure.GPIO_Pin = LED1_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED1_PORT, &GPIO_InitStructure);
    //LED 2
    GPIO_InitStructure.GPIO_Pin = LED2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED2_PORT, &GPIO_InitStructure);
    //LED 3
    GPIO_InitStructure.GPIO_Pin = LED3_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED3_PORT, &GPIO_InitStructure);
    //LED 4
    GPIO_InitStructure.GPIO_Pin = LED4_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED4_PORT, &GPIO_InitStructure);
    while(1)
    {
        GPIO_SetBits(LED1_PORT, LED1_PIN);  // set pin high
        delay(2000000);
        GPIO_ResetBits(LED1_PORT, LED1_PIN);    // set pin low
        delay(2000000);

        GPIO_SetBits(LED2_PORT, LED2_PIN);  // set pin high
        delay(2000000);
        GPIO_ResetBits(LED2_PORT, LED2_PIN);    // set pin low
        delay(2000000);

        GPIO_SetBits(LED3_PORT, LED3_PIN);  // set pin high
        delay(2000000);
        GPIO_ResetBits(LED3_PORT, LED3_PIN);    // set pin low
        delay(2000000);

        GPIO_SetBits(LED4_PORT, LED4_PIN);  // set pin high
        delay(2000000);
        GPIO_ResetBits(LED4_PORT, LED4_PIN);    // set pin low
        delay(2000000);
    }
    //return 0;
}



void delay(unsigned long count)
{
    while(count--);
}
