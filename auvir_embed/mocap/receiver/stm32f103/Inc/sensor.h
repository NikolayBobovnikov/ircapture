#ifndef SENSOR_HUB_H
#define SENSOR_HUB_H

#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"

#include <stdbool.h>

typedef enum {
    MSG_TYP_NONE,
    MSG_TYP_REQUEST_SENSOR_ID
} MessageTypeDef;

typedef struct SENSOR_DATA_MSG_t{
    uint8_t id_sensor;
    uint8_t id_hub;
    uint16_t angle_code;
} SENSOR_DATA_MSG_t;


void send_message();
void process_received_message();

void request_id();
void send_data();
void receive_uart_msg();
bool this_sensor_next();

#endif //SENSOR_HUB_H
