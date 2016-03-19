#include <stdbool.h>
#include "stm32f1xx_hal.h"

#define DELAY_PRESCALER (72 - 1)

void debug_init_gpio();
void init_gpio_led();
void delay_us(uint16_t delay);
void delay_general(uint16_t prescaler, uint16_t delay);
