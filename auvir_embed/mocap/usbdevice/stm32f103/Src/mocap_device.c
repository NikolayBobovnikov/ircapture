#include "mocap_device.h"

/// Interface with radio ================================
extern uint8_t rx_buf[TX_PLOAD_WIDTH];
extern uint8_t tx_buf[TX_PLOAD_WIDTH];
extern RadioMessage rx_message;
extern RadioMessage tx_message;
extern NRF_Module default_module;
extern NRF_Module data_module;

/// Interface with USB host ================================
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

/// usb device ================================
uint8_t UsbDeviceID = 0;
uint8_t UsbDeviceAddress[5] = {0};
bool is_beamer_registration_complete = false;

typedef struct RadioDevInfo{
    uint8_t ID;// used to check if Beamer is registered and not lost connection
    uint8_t Address[TX_ADR_WIDTH];
}RadioDevInfo;


/// Description of data:
/// ID of Sensor/Beamer = array index
/// Sensor/Beamer with corresponding ID registered <=> ID of array's element at (index==ID) != 0
/// Sensor/Beamer with corresponding ID is lost connection <=> ID of array's element at (index==ID) == 0 for  index==ID <= LastBeamerID
/// All Sensors/Beamers with ID > LastBeamerID has ID == 0 and not registered
/// Element at index 0 is special

// Beamers
#define MAX_BEAMER_NUM 256 // TODO
// TODO: change beamer_ids to bitset (so to store 256 bits will need 32 bytes, instead of 256)
#define MAX_BEAMER_IDS (256/8)
RadioDevInfo registered_beamers[MAX_BEAMER_NUM] = {0};
uint8_t last_beamer_id = 0;

// Sensors
#define MAX_SENSOR_NUM 256 // TODO
// TODO: change sensor_ids to bitset (so to store 256 bits will need 32 bytes, instead of 256)
#define MAX_SENSOR_IDS (256/8)
RadioDevInfo registered_sensors[MAX_SENSOR_NUM] = {0};
uint8_t last_sensor_id = 0;

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
void process_beamer_registration_request();
void process_sensor_registration_request();

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
    // get type of the message to determine its content and how to process it
    RM_Typ_e type = radio_get_msgtype(rx_message.type);

    //TODO: remove blinking
    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);

    switch (type) {
        case Typ_SensorData:
            process_sensor_data();
            break;
        case Typ_BeamerRequestRegistration:
            process_beamer_registration_request();
            break;
        case Typ_SensorRequestRegistration:
            process_sensor_registration_request();
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
        if(is_usb_configured()){
            CDC_Transmit_FS((uint8_t*)&(rx_message.data), TX_PLOAD_WIDTH);
        }
    }
}

void process_beamer_registration_request()
{
    // Determine source of packet
    RM_WhoAmI_e whoami = radio_get_whoami(rx_message.type);
    if (whoami != WhoAmI_Beamer){
        return;
    }

    //check beamers which lost connection

    //special case: first registration
    if(last_beamer_id == 0){
        //no beamers has been registered yet
        //increment last beamer id
        ++last_beamer_id;
        //assign ID and address
        registered_beamers[last_beamer_id].ID = last_beamer_id;

        // TODO FIXME: verify it works
        memcpy(&(registered_beamers[last_beamer_id].Address[0]), &(rx_message.data[0]), TX_ADR_WIDTH);
    }
    else for (uint8_t index = 1; index <= last_beamer_id; ++index){
        if(registered_beamers[index].ID == 0){
            //this beamer is in the list of lost connection
            //assign its ID new beamer
            registered_beamers[index].ID = index;
            //no need to assign address - just use existent address from beamer/sensor which lost connection
            // TODO FIXME: verify it works
            // stop registration procedure
            break;
        }
        // if no beamer whic lost connection was reused, register new beamer
        // increment last beamer id, assign it to new beamer
        ++last_beamer_id;
        registered_beamers[last_beamer_id].ID = last_beamer_id;
        // TODO FIXME: verify it works
        memcpy(&(registered_beamers[last_beamer_id].Address[0]), &(rx_message.data[0]), TX_ADR_WIDTH);
    }


    // broadcast beamer ID
    // TODO
}

void process_sensor_registration_request()
{
    // Determine source of packet
    RM_WhoAmI_e whoami = radio_get_whoami(rx_message.type);
    if(whoami != WhoAmI_Sensor) {
        return;
    }

    //register sensor
    last_sensor_id++;
    registered_sensors[last_sensor_id].ID = last_sensor_id;
    // TODO FIXME: use actual beamer info
    RadioDevInfo new_sensor_info;
    memcpy(&(registered_sensors[last_sensor_id].Address[0]), &(new_sensor_info.Address[0]), TX_ADR_WIDTH);
    // broadcast sensor ID
    // TODO
}


