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

 while(1)

 {

  GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_SET );
  GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_RESET);
  delay(1000000);
  GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_RESET );
  GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_SET);
  delay(1000000);
 }

}
