#include "sensor_hub.h"

extern UART_HandleTypeDef huart1;

static uint8_t this_hub_id = 0;
#define  max_number_of_sensors 128
static uint16_t registered_sensors[max_number_of_sensors] = {0};

static uint8_t SensorHubState = STARTING;
static SENSOR_DATA_MSG_t p_uart_msg = {0};
static HAL_StatusTypeDef status = HAL_OK;


///====================== Functions ======================

void sensorhub_process_states()
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

    sensorhub_receive_uart_msg();
    sensorhub_send_msg();

}

void sensorhub_receive_uart_msg()
{
    //TODO
}
void sensorhub_send_msg()
{
    //TODO
}



