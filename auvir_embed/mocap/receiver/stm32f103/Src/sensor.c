#include "sensor.h"


///====================== Variables ======================
static HAL_StatusTypeDef status = HAL_OK;
static MessageTypeDef msg_type = MSG_TYP_NONE;
static SENSOR_DATA_MSG_t p_uart_msg = {0};

static uint8_t assigned_hub_id = 0;
static uint8_t this_sensor_id = 0;
static uint8_t last_sensor_id = 0;
static const uint8_t first_sensor_id = 0;

///====================== Functions ======================
bool sensor_this_sensor_next(){

    // if received message from last registered sensor, start from beginning
    if(p_uart_msg.id_sensor == last_sensor_id){
        return this_sensor_id == first_sensor_id;
    }
    else{
        if(p_uart_msg.id_sensor == this_sensor_id - 1);
    }
}
