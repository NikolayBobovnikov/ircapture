#include <stdbool.h>
#include "stm32f1xx_hal.h"
#include "mxconstants.h"

#define DELAY_PRESCALER (72 - 1)

#ifndef LOW
#define LOW GPIO_PIN_RESET
#endif
#ifndef HIGH
#define HIGH GPIO_PIN_SET
#endif


void delay_us(uint16_t delay);
void delay_cycles(uint16_t delay);
void delay_timer_general(uint16_t prescaler, uint16_t delay);

void blink(uint8_t num_blinks, uint16_t delay_ms);
