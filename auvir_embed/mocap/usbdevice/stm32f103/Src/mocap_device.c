#include "mocap_device.h"

/// ================================ Interface with radio ================================================================
extern const RadioAddress Sensor_Beamer_DefaultAddress;
extern uint8_t rx_buf[TX_PLOAD_WIDTH];
extern uint8_t tx_buf[TX_PLOAD_WIDTH];
extern RadioMessage rx_message;
extern RadioMessage tx_message;
extern NRF_Module default_module;
extern NRF_Module data_module;

/// ================================ Interface with USB host ================================================================
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

// TODO FIXME: connect size of USB_Message with APP_RX_DATA_SIZE
typedef struct USB_Message{
    USBMessageType msgType;
    uint8_t data[31];
} USB_Message;

USB_Message tx_usb_message = {0};
USB_Message rx_usb_message = {0};


/// ================================ USB device ================================================================
uint8_t UsbDeviceID = 0;
RadioAddress UsbDeviceAddress = {0};
bool is_beamer_registration_complete = false;
bool is_already_receiving_data = false;
// this id for tracking sensors which lost connection
uint8_t expected_sensor_id = 0;

// stuff below is used to convert uint8_t* data to specific format, depending on message type
extern RadioDevInfo radiodevinfo;
extern SensorData sensordata;
extern uint16_t TmpID;
BeamerData beamerdata = {0};
IMUData imudata = {0};

/// Description of data:
/// ID of Sensor/Beamer = its index in the array
/// Sensor/Beamer with corresponding ID is registered <=> ID of array's element at (index==ID) != 0
/// Sensor/Beamer with corresponding ID is lost connection <=> ID of array's element at (index==ID) == 0 for  index==ID <= LastBeamerID
/// All Sensors/Beamers with ID > LastBeamerID are not registered and thus has ID == 0
/// Element at index 0 is special (can be used to check if list of registered devices is empty (LastID==0))

// Beamers
/// MAX_BEAMER_NUM should be greater or equal to MAX_BEAMERS_PER_SENSOR
#if MAX_BEAMERS_PER_SENSOR > 10
#define MAX_BEAMER_NUM MAX_BEAMERS_PER_SENSOR
#else
#define MAX_BEAMER_NUM 10
#endif

// TODO: change beamer_ids to bitset (so to store 256 bits will need 32 bytes, instead of 256)
#define MAX_BEAMER_IDS (MAX_BEAMER_NUM/8) // one bit per beamer
RadioDevInfo registered_beamers[MAX_BEAMER_NUM] = {0};
uint8_t last_beamer_id = 0;
uint8_t assigned_beamer_id = 0;

// Sensors
#define MAX_SENSOR_NUM 10 // TODO
// TODO: change sensor_ids to bitset (so to store 256 bits will need 32 bytes, instead of 256)
#define MAX_SENSOR_IDS (MAX_SENSOR_NUM/8) // one bit per sensor
RadioDevInfo registered_sensors[MAX_SENSOR_NUM] = {0};
uint8_t last_sensor_id = 0;
uint8_t assigned_sensor_id = 0;

#if 0
// ==================== for beamer
uint8_t my_id = 0;
uint8_t usbdevice_id = 0;
uint8_t USBDEVICE_ADDRESS[TX_ADR_WIDTH] = {0x10,0x20,0x30,0xab,0xab};
extern const uint8_t SENSORBEAM_DEFAULT_ADDRESS[TX_ADR_WIDTH];
uint32_t delay_after_sync_signal = 0;

// ==================== for sensor
uint8_t my_id = 0;
uint8_t usbdevice_id = 0;
uint8_t last_beamer_id = 0;
uint8_t USBDEVICE_ADDRESS[TX_ADR_WIDTH] = {0x10,0x20,0x30,0xab,0xab};
extern const uint8_t SENSORBEAM_DEFAULT_ADDRESS[TX_ADR_WIDTH];
uint32_t delay_after_sync_signal = 0;
#endif

/// ============================== Function declarations ==============================

void register_usb_device();
void nrf_receive_callback();
void process_sensor_data();
void process_beamer_registration_request();
void process_sensor_registration_request();

void send_assigned_radiodevinfo_back();
void start_receiving_sensor_data();

// registration
void try_register_beamer();
bool try_reassign_beamer();
void register_new_beamer_in_array();

void try_register_sensor();
void try_reassign_sensor();
void register_new_sensor_in_array();

// signals
void broadcast_startbeaming_sync_signal();
void send_reset_signal(RM_Dest_e dest, uint8_t id);

// processing data
void update_lost_beamers_from_sensordata();

/// ============================== Function definitions ==============================

// Registering of usb device in usb host
void register_usb_device()
{
    bool UsbHostDone = false;//TODO
    //Send registration request to usb host
    //Get UsbDeviceID and Address
    if(UsbHostDone){
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
                RadioDevInfo * reg_info = (RadioDevInfo * )&(rx_usb_message.data);
                reg_info->id;
                reg_info->address;
                reg_info->radio_channel;
            }
        }
    }
    else{ //mock routine
        UsbDeviceID = 1;
        UsbDeviceAddress.byte_array[0] = 11;
        UsbDeviceAddress.byte_array[1] = 22;
        UsbDeviceAddress.byte_array[2] = 33;
        UsbDeviceAddress.byte_array[3] = 44;
        UsbDeviceAddress.byte_array[4] = 55;
    }
}

// process message received from radiochannel depending on its type (header)
void nrf_receive_callback()
{
    //TODO: remove blinking
    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);

    // get type of the message to determine its content and how to process it
    RM_Typ_e type = radio_get_msgtype();

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

// process message if message type is "sensor data"
void send_sensor_data_to_usb_host(SensorData* snsrdata)
{
    if(is_usb_configured()){
        CDC_Transmit_FS((uint8_t*)snsrdata, sizeof(SensorData));
    }
}

// receive data from sensor, either beamerdata of imudata
void process_sensor_data()
{
    // Determine source of packet
    RM_WhoAmI_e whoami = radio_get_whoami();

    // verify that sender of request is beamer
    if (whoami != WhoAmI_Sensor){
        return;
    }

    // Monitor sensors which lost connection (compare receiveid id with expected id)
    // TODO
    ++expected_sensor_id;

    uint8_t curr_sensor_id = radio_rx_get_id();
    if(expected_sensor_id < curr_sensor_id){
        // expected_sensor_id is lost;
        registered_sensors[expected_sensor_id].id = 0;
        expected_sensor_id = curr_sensor_id;
    }

    // get sensordata
    radio_rx_get_sensordata(&sensordata);

    // check type of data (beamer/sensor)
    // do stuff special for beamer data
    if(sensordata.datatype == SDT_BeamerData){
        // Check sensor's data to identify beamers which doens't work
        update_lost_beamers_from_sensordata();

        // If all sensors has sent data, send StartBeam sync signal
        if (curr_sensor_id == last_sensor_id){
            broadcast_startbeaming_sync_signal();
        }
    }

    // send data to usb host
    send_sensor_data_to_usb_host(&sensordata);

}

// process message if message type is "registration request from beamer"
void process_beamer_registration_request()
{
    // get beamer's temporary id
    radio_rx_get_tmp_id(&TmpID);

    // Determine source of packet
    RM_WhoAmI_e whoami = radio_get_whoami();

    // verify that sender of request is beamer
    if (whoami != WhoAmI_Beamer){
        return;
    }

    //Register beamer and save its array index to assigned_beamer_id
    try_register_beamer();

    // beamer has been registered <=> its ID > 0
    if(assigned_beamer_id > 0){
        // prepare radio message
        radio_tx_set_message_header(WhoAmI_UsbDevice, Dest_Beamer, Typ_RespondRegistration);
        radio_tx_set_radiodevinfo(&(registered_beamers[assigned_beamer_id]));

        // and send back to beamer
        send_assigned_radiodevinfo_back();
    }

    // return if beamer registration is not complete
    if(!is_beamer_registration_complete){
        return;
    }else{
        //otherwise, start working mode if not already there
        if(!is_already_receiving_data){
            start_receiving_sensor_data();
        }
    }

    // broadcast beamer ID - TODO
}

// process message if message type is "registration request from sensor"
void process_sensor_registration_request()
{
    // get sensor's temporary id
    radio_rx_get_tmp_id(&TmpID);

    // Determine source of packet
    RM_WhoAmI_e whoami = radio_get_whoami();

    // verify that sender of request is sensor
    if(whoami != WhoAmI_Sensor) {
        return;
    }

    // only register sensors if beamers are registered
    if(!is_beamer_registration_complete){
        return;
    }

    //Register sensor and save its array index to assigned_beamer_id
    try_register_sensor();

    // sensor has been registered <=> its ID > 0
    if(assigned_sensor_id > 0){
        // prepare radio message
        radio_tx_set_message_header(WhoAmI_UsbDevice, Dest_Sensor, Typ_RespondRegistration);
        radio_tx_set_radiodevinfo(&(registered_sensors[assigned_sensor_id]));

        // and send back to sensor
        send_assigned_radiodevinfo_back();
    }

}

// finish processing message if message type is "registration request" (beamer/sensor)
void send_assigned_radiodevinfo_back()
{
    TXX(&default_module);
}

// turn on data receiving radiomodule in rx mode
void start_receiving_sensor_data()
{
    //TODO
    // for now, use default radiomodule
}

// beamer registration routines
void try_register_beamer()
{
    //initialize ID to zero
    assigned_beamer_id = 0;

    if(last_beamer_id == 0){
        //no beamers has been registered yet
        // increment last beamer id, assign it to new beamer
        register_new_beamer_in_array();
    }
    else {
        //check if there are beamers that marked as lost connection, if so - reuse one of them
        try_reassign_beamer();
        //check if succeeded
        if(assigned_sensor_id == 0){
            // if there was no beamer which lost connection, and there is a place for new beamer, register new beamer
            if(last_beamer_id < MAX_BEAMER_NUM){
                register_new_beamer_in_array();
            }
        }
    }

    // beamer has been registered <=> its ID > 0
}
bool try_reassign_beamer()
{
    //initialize ID to zero
    assigned_beamer_id = 0;

    for (uint8_t index = 1; index <= last_beamer_id; ++index){
        if(registered_beamers[index].id == 0){
            //use its ID and address for new beamer (will send them to new beamer afterwards)
            assigned_beamer_id = index;
            registered_beamers[assigned_beamer_id].id = assigned_beamer_id;
            registered_beamers[assigned_beamer_id].prev_id = TmpID;

            //no need to change address (both beamer and usbdevice) - just use existent address from beamer/sensor which lost connection
            // TODO FIXME: verify that using existent address from the array works
            // stop registration procedure
            // TODO FIXME: verify that break
            return;
        }
    }
}
void register_new_beamer_in_array()
{
    // increment last beamer id, assign it to new beamer
    ++last_beamer_id;
    assigned_beamer_id = last_beamer_id;

    registered_beamers[assigned_beamer_id].id = assigned_beamer_id;
    registered_beamers[assigned_beamer_id].prev_id = TmpID;
    registered_beamers[assigned_beamer_id].id_usbdevice = UsbDeviceID;

    // use default address for now
    memcpy(&(registered_beamers[assigned_beamer_id].address), &Sensor_Beamer_DefaultAddress, sizeof(RadioAddress));
    memcpy(&(registered_beamers[assigned_beamer_id].address_usbdevice), &UsbDeviceAddress, sizeof(RadioAddress));

}

// sensor registration routines
void try_register_sensor()
{
    //initialize ID to zero
    assigned_sensor_id = 0;

    if(last_sensor_id == 0){
        //no sensors has been registered yet
        // increment last sensor id, assign it to new beamer
        register_new_sensor_in_array();
    }
    else {
        //check if there are sensors that marked as lost connection, if so - reuse one of them
        try_reassign_sensor();

        //check if succeeded
        if(assigned_sensor_id == 0){
            // if there was no sensor which lost connection, and there is a place for new sensor, register new sensor
            if(last_sensor_id < MAX_SENSOR_NUM){
                register_new_sensor_in_array();
            }
        }
    }
    // sensor has been registered <=> its ID > 0
}
void try_reassign_sensor()
{
    //initialize ID to zero TODO: redundant?
    assigned_sensor_id = 0;

    // TODO FIXME: check that below works for last_sensor_id==0
    // it will allow to remove separate routine for first registration
    for (uint8_t index = 1; index <= last_sensor_id; ++index){
        if(registered_sensors[index].id == 0){
            //use its ID and address for new beamer (will send them to new beamer afterwards)
            assigned_sensor_id = index;
            registered_sensors[assigned_sensor_id].id = assigned_sensor_id;
            registered_sensors[assigned_sensor_id].prev_id = TmpID;

            //no need to change address - just use existent address from beamer/sensor which lost connection
            // TODO FIXME: verify that using existent address from the array works
            // stop registration procedure
            // TODO FIXME: verify that break
            return;
        }
    }
}
void register_new_sensor_in_array()
{
    // increment last beamer id, assign it to new beamer
    ++last_sensor_id;
    assigned_sensor_id = last_sensor_id;

    registered_sensors[assigned_sensor_id].id = assigned_sensor_id;
    registered_sensors[assigned_sensor_id].prev_id = TmpID;
    // TODO FIXME: verify it works
    // use default address for now
    memcpy(&(registered_sensors[assigned_sensor_id].address), &Sensor_Beamer_DefaultAddress, sizeof(RadioAddress));
    memcpy(&(registered_sensors[assigned_sensor_id].address), &UsbDeviceAddress, sizeof(RadioAddress));
}

void broadcast_startbeaming_sync_signal()
{
    // set header to tx_message
    radio_tx_set_message_header(WhoAmI_UsbDevice,Dest_Beamer,Typ_SyncBeamer);

    // transmit it using... datamodule? TODO - define which module to use
    // for now, use default module anyway
    TXX(&default_module);
}

void send_reset_signal(RM_Dest_e dest, uint8_t id){
    radio_tx_set_message_header(WhoAmI_UsbDevice, dest, Typ_Reset);
    radio_tx_set_id(id);

    //TODO: use data_module
    TXX(&default_module);
}

void update_lost_beamers_from_sensordata()
{
    //TODO FIXME: check that typecasting works
    //no deep copy, according to http://stackoverflow.com/questions/2302351/assign-one-struct-to-another-in-c#comment2268076_2302359
    //http://stackoverflow.com/questions/2302351/assign-one-struct-to-another-in-c
    beamerdata = *((BeamerData*)&sensordata.data[0]);
    for (uint8_t index = 0; index < MAX_BEAMERS_PER_SENSOR; ++index)
    {
        // check if actual beamer data is filled with zero
        // or fast way which is sufficient: check that beamer's ID is zero
        // TODO: this works for "motion" platform
        // TODO - how to determine beamer's ID in the future case of arena planform?
        // for "arena" platform its impossible to infere beamer id from beamer_data_array index
        if(beamerdata.beamer_data_array[index].beamer_id == 0)
        {
            // mark beamer as one which 'lost connection' or stopped to work
            registered_beamers[index].id = 0;

            // reset this beamer
            send_reset_signal(Dest_Beamer, index);
        }
    }
}





