#include "mocap_device.h"

/// Interface with radio
extern uint8_t rx_buf[TX_PLOAD_WIDTH];
extern uint8_t tx_buf[TX_PLOAD_WIDTH];
extern RadioMessage rx_message;
extern RadioMessage tx_message;
extern NRF_Module default_module;
extern NRF_Module data_module;

/// Interface with USB host
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
extern uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];
bool received_usb_host_responce = false;

typedef enum USBMessageType{
    USB_None,
    USB_DeviceRegistrationRequest,
    USB_DeviceRegistrationInfo,
    USB_BeamerRegistrationInfo,
    USB_SensorRegistrationInfo,
    USB_Command,
} USBMessageType;

typedef struct RegistrationInfo{
    uint8_t ID;
    uint8_t radiochannel;
    uint8_t address[TX_PLOAD_WIDTH];
} RegistrationInfo;


// TODO FIXME: connect size of USB_Message with APP_RX_DATA_SIZE
typedef struct USB_Message{
    USBMessageType msgType;
    uint8_t data[31];
} USB_Message;

USB_Message tx_usb_message = {0};
USB_Message rx_usb_message = {0};

// ==================== for usb device
uint8_t UsbDeviceID = 0;
uint8_t UsbDeviceAddress[5] = {0};

// Beamers
#define MAX_BEAMER_NUM 10 // TODO
uint8_t beamer_ids[MAX_BEAMER_NUM] = {0};
uint8_t last_beamer_id = 0;

// Sensors
#define MAX_SENSOR_NUM 10 // TODO
uint8_t sensor_ids[MAX_SENSOR_NUM] = {0};
uint8_t last_sensor_id = 0;

bool is_beamer_registration_complete = false;

// ==================== for beamer
uint8_t my_id = 0;
uint8_t usbdevice_id = 0;
uint8_t USBDEVICE_ADDRESS[TX_ADR_WIDTH] = {0x10,0x20,0x30,0xab,0xab};
extern uint8_t SENSORBEAM_DEFAULT_ADDRESS[TX_ADR_WIDTH];
uint32_t delay_after_sync_signal = 0;

// ==================== for sensor
#if 0
uint8_t my_id = 0;
uint8_t usbdevice_id = 0;
uint8_t last_beamer_id = 0;
uint8_t USBDEVICE_ADDRESS[TX_ADR_WIDTH] = {0x10,0x20,0x30,0xab,0xab};
uint32_t delay_after_sync_signal = 0;
#endif

/// ============================== Function declarations ==============================

void register_usb_device();
void nrf_receive_callback();
void process_sensor_data();
void process_registration_request();

/// ============================== Function definitions ==============================

void register_usb_device()
{
#define usb_host_done 0

#if usb_host_done
    //Send registration request to usb host
    //
    USB_Message msg = {0};
    msg.msgType = USB_DeviceRegistrationRequest;

    if(is_usb_configured()){
        CDC_Transmit_FS((uint8_t*)&msg, APP_RX_DATA_SIZE);

        // wait for host responce
        while(!received_usb_host_responce){}

        //get usb device ID, usb device address
        memcpy(&rx_usb_message, &UserRxBufferFS[0], APP_RX_DATA_SIZE);

        if(rx_usb_message.msgType == USB_DeviceRegistrationInfo){
            RegistrationInfo * reg_info = (RegistrationInfo * )&(rx_usb_message.data);
            reg_info->ID;
            reg_info->address;
            reg_info->radiochannel;
        }
    }
#else //mock routine
    UsbDeviceID = 1;
    UsbDeviceAddress[0] = 11;
    UsbDeviceAddress[1] = 22;
    UsbDeviceAddress[2] = 33;
    UsbDeviceAddress[3] = 44;
    UsbDeviceAddress[4] = 55;

#endif
}

void nrf_receive_callback()
{
    uint8_t t = 0;
    radio_set_msgtype(&t, Typ_SensorData);

    // get type of the message to determine its content and how to process it
    //RM_Typ_e type = radio_get_msgtype(rx_message.type);
    RM_Typ_e type = radio_get_msgtype(t);

    switch (type) {
        case Typ_SensorData:
            process_sensor_data();
            break;
        case Typ_RequestRegistration:
            process_registration_request();
            break;
        default:
            break;
    }

}

void process_sensor_data()
{
    //TODO: remove
    is_beamer_registration_complete = true;

    if(is_beamer_registration_complete){
        // process data
        // TODO
        HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
        if(is_usb_configured()){
            CDC_Transmit_FS((uint8_t*)&(rx_message.data), TX_PLOAD_WIDTH);
        }
    }
}

void process_registration_request()
{
    // Determine source of packet
    RM_WhoAmI_e whoami = radio_get_whoami(rx_message.type);
    switch (whoami) {
        case WhoAmI_UsbDevice:
            //do nothing
            //TODO: check for redundancy
            break;
        case WhoAmI_Sensor:
            //register sensor
            last_sensor_id++;
            sensor_ids[last_sensor_id] = last_sensor_id;
            // broadcast sensor ID
            // TODO
            break;
        case WhoAmI_Beamer:
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
            break;
        default:
            break;
    }
}


