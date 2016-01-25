#include "sensor_hub.h"



uint16_t hub_id;
#define  max_number_of_sensors 128
uint16_t registered_sensors[max_number_of_sensors] = {0};

// states
enum{
    STARTING,
    REQUESTING_HUB_ID,
    WAITING_FOR_START_REGISTRATION,
    REGISTRATION_OPEN,
    REGISTRATION_CLOSED
};

uint8_t SensorHubState = STARTING;


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
