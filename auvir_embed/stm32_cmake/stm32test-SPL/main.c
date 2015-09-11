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

inline void init_led()
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

inline void blue_led_on()
{
    GPIOC->BSRR |= GPIO_BSRR_BS8;
}

inline void blue_led_off()
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
    GPIOA->CLR &= (~GPIO_CRL_CNF0);
    GPIOA->CLR |= GPIO_CRL_CNF0_1;

    //

    init_led();

    while(1)
    {
        unsigned int input_data_register = GPIO_IDR_IDR0;
        if( GPIO_IDR_IDR0 )
        {
            blue_led_off();
        }
        else
        {
            blue_led_on();
        }
    }
}


