#include "stm32f10x.h"
//#include "stm32f10x_conf.h"

const char* const example_string = "Hello World!\r\n\0";

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

void init_button()
{
	// enable clock for port A (where button is located)
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	// setup button
	// input push/pull
	GPIOA->CRL &= (~GPIO_CRL_MODE0);
	GPIOA->CRL &= (~GPIO_CRL_CNF0);
	GPIOA->CRL |= GPIO_CRL_CNF0_1;
}

void init_uart()
{

	/* Initialize GPIO for transmit/receive pin */

    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    //Transmit register - pin 9
    //MODE: 11: Output mode, max speed 50 MHz.
    GPIOA->CRH |= GPIO_CRH_MODE9;

    //10: Alternate function output Push-pull
    GPIOA->CRH &= (~GPIO_CRH_CNF9);
    GPIOA->CRH |= GPIO_CRH_CNF9_1;

    //Receive register - pin 10
    // MODE: 00: Input mode (reset state)
    GPIOA->CRH &= (~GPIO_CRH_MODE10);
    // CNF: 01: Floating input (reset state)
    GPIOA->CRH &= (~GPIO_CRH_CNF10);
    GPIOA->CRH |= GPIO_CRH_CNF10_0;


    /* Initialize uart peripheral*/

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


        USART1->CR1 |= USART_CR1_RE;


}

void usart_sent_byte(uint8_t ch)
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
  while ((USART1->SR & USART_SR_TC) == 0);

  // wait for TX
     //while ((USART1->SR & USART_SR_TXE) == 0);
}

void usart_send_str(uint8_t * str)
{
	while(*str != '\0')
	{
		usart_sent_byte(*str);
		str++;
	}
}

void button_led()
{

    init_led();
    init_button();
    init_uart();

    while(1)
    {
        unsigned int is_button_off = (GPIOA->IDR & 0x1);
        if( is_button_off == 0)
        {
            blue_led_off();
        }
        else
        {
            blue_led_on();
            usart_send_str((uint8_t *)example_string);
        }
    }
}


int main()
{
    button_led();
    return 0;
}
