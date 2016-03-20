#ifndef SENSOR_HUB_H
#define SENSOR_HUB_H

#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"

#include <stdbool.h>

typedef struct SENSOR_DATA_MSG_t{
    uint8_t id_sensor;
    uint8_t id_hub;
    uint16_t angle_code;
} SENSOR_DATA_MSG_t;

// states
enum{
    NONE,
    STARTING,
    REQUESTING_HUB_ID,
    WAITING_FOR_START_REGISTRATION,
    REGISTRATION_OPEN,
    REGISTRATION_CLOSED
};


void sensorhub_process_states();
void sensorhub_receive_uart_msg();
void sensorhub_send_msg();



#endif //SENSOR_HUB_H
