#include "mocap_device.h"

/// Interface with radio
extern uint8_t rx_buf[TX_PLOAD_WIDTH];
extern uint8_t tx_buf[TX_PLOAD_WIDTH];
extern RadioMessage rx_message;

// ==================== for usb device
// Beamers
#define MAX_BEAMER_NUM 10 // TODO
uint8_t beamer_ids[MAX_BEAMER_NUM] = {0};
uint8_t last_beamer_id = 0;

// Sensors
#define MAX_SENSOR_NUM 10 // TODO
uint8_t sensor_ids[MAX_SENSOR_NUM] = {0};
uint8_t last_sensor_id = 0;

UsbDeviceStates usb_state = USBState_None;
bool is_beamer_registration_complete = false;
bool is_sensor_registration_complete = false;

// ==================== for beamer
uint8_t my_id = 0;
uint8_t last_id = 0;
uint8_t usbdevice_id = 0;
uint8_t USBDEVICE_ADDRESS[TX_ADR_WIDTH] = {0x10,0x20,0x30,0xab,0xab};
extern uint8_t BROADCAST_DEFAULT_ADDRESS[TX_ADR_WIDTH];
uint32_t delay_after_sync_signal = 0;

// ==================== for sensor
#if 0
uint8_t my_id = 0;
uint8_t last_id = 0;
uint8_t usbdevice_id = 0;
uint8_t USBDEVICE_ADDRESS[TX_ADR_WIDTH] = {0x10,0x20,0x30,0xab,0xab};
uint32_t delay_after_sync_signal = 0;
#endif



void process_sensor_data()
{
    if(is_sensor_registration_complete && is_beamer_registration_complete){
        // process data
        // TODO
        HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
        if(is_usb_configured()){
            CDC_Transmit_FS(&(rx_message.data), TX_PLOAD_WIDTH);
        }
    }

}
void process_registration_request()
{
    uint8_t msg_type_byte = rx_message.type;
    // who am i - sensor or beamer or another usb device
    if(is_sensor(msg_type_byte)){
        //register sensor
        if(!is_sensor_registration_complete){
            last_sensor_id++;
            sensor_ids[last_sensor_id] = last_sensor_id;
            if(last_sensor_id == MAX_SENSOR_NUM){
                is_sensor_registration_complete = true;
            }
        }
        // broadcast sensor ID
        // TODO

    }
    else if(is_beamer(msg_type_byte)){
        //register beamer
        if(!is_beamer_registration_complete){
            last_beamer_id++;
            beamer_ids[last_beamer_id] = last_beamer_id;
            if(last_beamer_id == MAX_BEAMER_NUM){
                is_beamer_registration_complete = true;
            }
        }
        // broadcast beamer ID
        // TODO
    }
    else if(is_usb_device(msg_type_byte)){
        //do nothing
        //TODO: check for redundancy
    }

}
void nrf_receive_callback()
{

    uint8_t msg_type_byte = rx_message.type;

    // purpose of the packet
    if(is_sensor_data(msg_type_byte)){
        process_sensor_data();
    }
    else if(is_reg_request(msg_type_byte)){
        process_registration_request();
    }
    // check sync signal
    else if(is_sync_beamer(msg_type_byte)){

    }
    else if(is_sync_sensor(msg_type_byte)){

    }

}

