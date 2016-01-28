#ifndef BEAMER_HUB_H
#define BEAMER_HUB_H

#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"


typedef enum{
    WHOAMI_UNDEFINED,
    SENSOR,
    SENSOR_HUB,
    BEAMER,
    BEAMER_HUB
} WHOAMI_t;

typedef enum{
    UART_MSG_UNDEFINED,
    UART_MSG_REQUEST_ID,
    UART_MSG_RESPONCE_ID,
    UART_MSG_RESPONCE_REGISTRATION_CLOSED
} MSG_t;

typedef struct UART_FRAME_t{
    MSG_t message_type;
    WHOAMI_t who_am_i;
    uint8_t data_size;
}UART_FRAME_t;

void process_beamer_hub_states();


#endif //BEAMER_HUB_H
