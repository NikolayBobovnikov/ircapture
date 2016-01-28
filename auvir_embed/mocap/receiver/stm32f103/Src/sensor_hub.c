#include "sensor_hub.h"

extern UART_HandleTypeDef huart1;

uint8_t id_hub = 0;
uint8_t id_sensor = 0;
#define  max_number_of_sensors 128
uint16_t registered_sensors[max_number_of_sensors] = {0};

uint8_t SensorHubState = STARTING;
SENSOR_MSG_t p_uart_msg = {0};
HAL_StatusTypeDef status = HAL_OK;


///====================== Functions ======================

void process_sensor_hub_states()
{
    switch(SensorHubState){
        case STARTING:
            break;
        case REQUESTING_HUB_ID:
            break;
        case WAITING_FOR_START_REGISTRATION:
            break;
        case REGISTRATION_OPEN:
            break;
        case REGISTRATION_CLOSED:
            break;
        default:
            break;
    }
}

void main_loop(){

    receive_uart_msg();
    if(this_sensor_next()){
        send_msg();
    }
}

void receive_uart_msg(){
    status = HAL_UART_Receive(&huart1, (uint8_t *)&p_uart_msg, sizeof(p_uart_msg), 1000);
}

bool this_sensor_next(){

    // if received message from last registered sensor, start from beginning
    if(p_uart_msg.id_sensor == last_id_in_array){
        return id_sensor == first_id_in_array;
    }
    else{
        if(p_uart_msg.id_sensor == id_sensor - 1)
    }
}
