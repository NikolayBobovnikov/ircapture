#include "stm32f10x.h" 
#include "stm32f10x_conf.h" 

void _exit(void)
{
    while(1)
    {
        ;
    }
}
void delay(unsigned int ms)
{
    unsigned int i;
  for (i = 0; i < ms; ++i)
  {
      ;
  }

 
}
int main(void)
{
/*
 GPIO_InitTypeDef gpio;
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
 GPIO_StructInit(&gpio);
 gpio.GPIO_Pin = GPIO_Pin_9; // Green LED
 gpio.GPIO_Mode = GPIO_Mode_Out_PP;
 gpio.GPIO_Speed = GPIO_Speed_2MHz;
 GPIO_Init(GPIOC, &gpio);
 gpio.GPIO_Pin = GPIO_Pin_8; // Blue LED
 gpio.GPIO_Mode = GPIO_Mode_Out_PP;
 gpio.GPIO_Speed = GPIO_Speed_2MHz;
 GPIO_Init(GPIOC, &gpio);
*/

    //====================================
    // enable clock for gpio port C
    //====================================
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

    //====================================
    // setup GPIO port C pin 8 for output
    //====================================

    // output mode push-pull - by default
    //GPIOC->CRH |= (0 << 2);
    //GPIOC->CRH |= (0 << 3);

    // setup speed 2Mhz
    GPIOC ->CRH &= ~GPIO_CRH_MODE8;
    GPIOC ->CRH |= GPIO_CRH_MODE8_1;
    GPIOC ->CRH &= ~GPIO_CRH_CNF8;

    // gpio on
    GPIOC->BRR |= GPIO_BRR_BR8;
    GPIOC->BSRR |= GPIO_BSRR_BS8;
    //GPIOC->BSRR = (1 << 8);

 while(1)

 {

/*
  GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_SET );
  GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_RESET);
  delay(1000000);
  GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_RESET );
  GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_SET);
  delay(1000000);
*/
GPIOC->BSRR |= GPIO_BSRR_BS8;
delay(1000000);
GPIOC->BRR |= GPIO_BRR_BR8;
delay(1000000);
 
}

}
