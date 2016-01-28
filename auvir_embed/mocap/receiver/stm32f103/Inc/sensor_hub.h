#ifndef SENSOR_HUB_H
#define SENSOR_HUB_H

#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"

#include <stdbool.h>

typedef struct SENSOR_MSG_t{
    uint8_t id_sensor;
    uint8_t id_hub;
    uint16_t data;
} SENSOR_MSG_t;

// states
enum{
    NONE,
    STARTING,
    REQUESTING_HUB_ID,
    WAITING_FOR_START_REGISTRATION,
    REGISTRATION_OPEN,
    REGISTRATION_CLOSED
};


void process_sensor_hub_states();
void receive_uart_msg();
bool this_sensor_next();

#endif //SENSOR_HUB_H
