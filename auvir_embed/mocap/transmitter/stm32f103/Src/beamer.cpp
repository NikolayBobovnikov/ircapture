#include "beamer.h"
#include <stdbool.h>


/// ================== Parameters ================
extern NRF_Module default_module;
extern NRF_Module data_module;

/// ================== Variables ================


/// ============================== Function declarations ==============================
void nrf_receive_callback();

/// ============================== Function definitions ==============================
// process message received from radiochannel depending on its type (header)
void nrf_receive_callback()
{
    //TODO: remove blinking
    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);

    // get type f the message to determine its content and how to process it
    RM_Typ_e type = radio_get_msgtype();

    switch (type) {
    case Typ_SensorData:
        //process_sensor_data();
        break;
    case Typ_BeamerRequestRegistration:
        //process_beamer_registration_request();
        break;
    case Typ_SensorRequestRegistration:
        //process_sensor_registration_request();
        break;
    default:
        break;
    }

}
