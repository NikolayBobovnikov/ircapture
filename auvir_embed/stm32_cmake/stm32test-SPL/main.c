#include "stm32f10x.h" 

void _exit(void)
{
    while(1)
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
 GPIO_Init(GPIOC, &gpio);

 while(1)

 {

  static int count=0;

  static int i;

  static int led_state=0;



  for (i=0; i<1000000; ++i)
  {
      ;
  }

  GPIO_WriteBit(GPIOC, GPIO_Pin_9, led_state ? Bit_SET : Bit_RESET);

     led_state = !led_state;

     GPIO_WriteBit(GPIOC, GPIO_Pin_8, led_state ? Bit_SET : Bit_RESET);




 }

}
