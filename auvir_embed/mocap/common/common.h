#include <stdbool.h>
#include "stm32f1xx_hal.h"

#define DELAY_PRESCALER (72 - 1)


void delay_us(uint16_t delay);
void delay_cycles(uint16_t delay);
void delay_timer_general(uint16_t prescaler, uint16_t delay);
