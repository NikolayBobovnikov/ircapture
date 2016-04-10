#include "stm32f1xx_hal.h"
#include <stdbool.h>

// typedef struct RadioMessage RadioMessage; TODO
typedef struct RadioMessage{
    uint8_t type;
    uint8_t data[30];
    const uint8_t check_zero;
}RadioMessage;

void nrf24_setup_modules_gpio();
