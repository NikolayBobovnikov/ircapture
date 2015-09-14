#include "stm32f10x.h" 
#include "stm32f10x_conf.h" 

void _exit(void)
{
    while(1)
    {
        ;
    }
}

void delay(unsigned int num)
{
    unsigned int i;
    for (i = 0; i < num; i++)
    {
        ;
    }
}

void init_led()
{
    //====================================
    // enable clock for gpio port C
    //====================================
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

   //====================================
    // setup GPIO port C pin 8 for output
    //====================================
    // output mode push-pull - by default
    // setup speed 2Mhz
    GPIOC->CRH &= (~GPIO_CRH_CNF8);
    GPIOC->CRH |= GPIO_CRH_MODE8_1;

}

void blue_led_on()
{
    GPIOC->BSRR |= GPIO_BSRR_BS8;
}

void blue_led_off()
{
    GPIOC->BRR |= GPIO_BRR_BR8;
}


void blink_led()
{

    init_led();

    while(1)
    {
        blue_led_on();
        delay(1000000);

        blue_led_off();
        delay(1000000);
    }
}

void button_led()
{
    // enable clock for port A (where button is located)
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    // setup button
    // input push/pull
    GPIOA->CRL &= (~GPIO_CRL_MODE0);
    GPIOA->CRL &= (~GPIO_CRL_CNF0);
    GPIOA->CRL |= GPIO_CRL_CNF0_1;

    //

    init_led();
    init_uart();

    while(1)
    {
        unsigned int input_data_register = (GPIOA->IDR & 0x1);
        if( input_data_register )
        {
            blue_led_off();
        }
        else
        {
            blue_led_on();
            USART_PutChar('A');
        }
    }
}

void init_uart()
{

    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN;


//1. Enable the USART by writing the UE bit in USART_CR1 register to 1.
    USART1->CR1 |= USART_CR1_UE;                // Uart Enable
//2. Program the M bit in USART_CR1 to define the word length.
    USART1->CR1 &= (~USART_CR1_M);              // 8 bit word - bit M is reset
//3. Program the number of stop bits in USART_CR2.
    USART1->CR2 &= (~USART_CR2_STOP);           // 1 stop bit - 00
/*4. Select DMA enable (DMAT) in USART_CR3 if Multi buffer Communication is to take
place. Configure the DMA register as explained in multibuffer communication.*/
    USART1->CR3 &= (~USART_CR3_DMAT);           // DMA disabled
//5. Select the desired baud rate using the USART_BRR register.
    USART1->BRR = (SystemCoreClock / 115200);   //115200 bps
//6. Set the TE bit in USART_CR1 to send an idle frame as first transmission.
    USART1->CR1 |= USART_CR1_TE;
}

void USART_PutChar(uint8_t ch)
{
/*7. Write the data to send in the USART_DR register (this clears the TXE bit). Repeat this
for each data to be transmitted in case of single buffer.*/

  /*
   * Before write data to DR register, we check state of TXE (transmitter empty) flag. When this flag is clear, we can't write to DR register.
   * High value of this flag mean that is no data in transmit buffer (previous data just be sent or there is first tranmission) and we can write new data to send.
  */
  while(!(USART1->SR & USART_SR_TXE))
  {
    ;
  }
  USART1->DR = ch;
/*8. After writing the last data into the USART_DR register, wait until TC=1. This indicates
that the transmission of the last frame is complete. This is required for instance when
the USART is disabled or enters the Halt mode to avoid corrupting the last
transmission.*/

}


int main()
{
    button_led();
    return 0;
}
