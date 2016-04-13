#ifndef RADIO_DATA_FORMATS_H
#define RADIO_DATA_FORMATS_H


/* This file is common for beamers, sensors and usbdevices
 * Here are described common formats of radio messages
 * and interfaces to work with them
*/

#include "stm32f1xx_hal.h"

#define TX_ADR_WIDTH    5   // 5 uint8_ts TX(RX) address width
#define TX_PLOAD_WIDTH  32  // 32 uint8_ts TX payload
#define RF_CHANNEL      40  //

// for single person on motion platform, sensor will receive angles from all beamers.
// In this case all registered beamers are nearby
// But for multi person with arena platform, sensor won't be able to receive angles from all beamers,
// Instead, each sensor should receive data from subset of beamers (e.g. which are nearby)
// So we need to limit number of beamers for each sensor to get data from
#define MAX_BEAMERS_PER_SENSOR 10 /// should be less or equal to MAX_BEAMER_NUM

typedef struct RadioAddress{
    uint8_t byte_array[TX_ADR_WIDTH];
} RadioAddress;


typedef struct RadioMessage{
    uint8_t header;
    uint8_t data[31];
    //Contents of data depends on the message type:
    //Typ_BeamerRequestRegistration
    //  uint16_t TmpBeamerID
    //Typ_SensorRequestRegistration
    //  uint16_t TmpSensorID
    //Typ_RespondRegistration
    //  RadioDevInfo
    //Typ_SensorData
    //  SensorData ? TODO to be defined
    //Typ_UpdateLastBeamerID
    //  uint8_t BeamerID
    //Typ_SyncBeamer
    //  uint8_t ID (beamer/sensor)
    //Typ_Reset
    //  uint8_t ID (beamer/sensor)
}RadioMessage;

typedef struct RadioDevInfo{
    uint8_t id;
    uint8_t radio_channel;
    RadioAddress address;

    uint16_t prev_id; //TODO: workaround this. Need extra space and processing for prev_id

    uint8_t id_usbdevice;
    RadioAddress address_usbdevice;
} RadioDevInfo;

typedef struct SingleBeamerData{
    // TODO: get rid of beamer id, use array index instead. Low priority
    uint8_t beamer_id;
    uint16_t angle;
}SingleBeamerData;


typedef struct BeamerData{
    SingleBeamerData beamer_data_array[MAX_BEAMERS_PER_SENSOR];
}BeamerData;

typedef struct IMUData{
    //accel
    uint16_t ax;
    uint16_t ay;
    uint16_t az;
    //gyro
    uint16_t gx;
    uint16_t gy;
    uint16_t gz;
    //magnet
    uint16_t mx;
    uint16_t my;
    uint16_t mz;
}IMUData;

typedef enum SensorDataType{
    SDT_None,
    SDT_IMUData,
    SDT_BeamerData
}SensorDataType;

//TODO: make sure size of SensorData is the same as size of radio buffer (rx/tx payload) if it is not dynamic
typedef struct SensorData{
    uint8_t sensor_id;
    SensorDataType datatype;
    uint8_t data[30];
    //Contents of data depends on the message type:
    //SDT_BeamerData:
    //BeamerData beamer_data[MAX_BEAMERS_PER_SENSOR]; //30 bytes
    //SDT_IMUData:
    //IMUData imu_data;                               //18 bytes
} SensorData;


/// ===================================== Radio message header =====================================
/// Notes:
/// Typ_Reset can be sent only from a usbdevice
/// RM_WhoAmI_e cannot be equal to RM_Dest_e
/// RM_WhoAmI_e probably not needed, RM_Dest_e is enough:
/// To Beamer/Sensor from only usbdevice; To Usbdevice from only Beamer/sensor
/// Redundancy is probably not bad, can do additional check

typedef enum RM_WhoAmI_e{
    WhoAmI_UsbDevice,
    WhoAmI_Sensor,
    WhoAmI_Beamer
}RM_WhoAmI_e;

typedef enum RM_Dest_e{
    Dest_UsbDevice,
    Dest_Sensor,
    Dest_Beamer
}RM_Dest_e;

typedef enum RM_Typ_e{
    Typ_BeamerRequestRegistration,
    Typ_SensorRequestRegistration,
    Typ_RespondRegistration,
    Typ_SensorData,
    Typ_UpdateLastBeamerID,
    Typ_SyncBeamer,
    Typ_Reset
}RM_Typ_e;


// Bit      Description     Value       Detailed description
// 0:1      WhoAmI(Src)     0   00      UsbDevice
//                          1   01      Sensor
//                          2   01      Beamer
//                          3   11      <preserved>
// 2:3      Dst             0   00      UsbDevice
//                          1   01      Sensor
//                          2   01      Beamer
//                          3   11      <preserved>
// 4:6      Typ             0   000     BeamerRequestRegistration
//                          1   001     SensorRequestRegistration
//                          2   010     RespondRegistration
//                          3   011     SensorData
//                          4   100     UpdateLastBeamerID
//                          5   101     SyncBeamer
//                          6   110     Reset
//                          7   111     <preserved>
// 7        <preserved>

#define RM_Mask_WhoAmI  0b00000011
#define RM_Mask_Dst     0b00001100
#define RM_Mask_Typ     0b01110000
#define RM_Bit_WhoAmI   0
#define RM_Bit_Dst      2
#define RM_Bit_Typ      4

// TODO: consider right shifting  (n >> k) & 1
// http://stackoverflow.com/questions/2249731/how-do-i-get-bit-by-bit-data-from-an-integer-value-in-c
RM_WhoAmI_e radio_get_whoami();
RM_Dest_e radio_get_dst();
RM_Typ_e radio_get_msgtype();

void radio_set_whoami(RM_WhoAmI_e whoami);
void radio_set_dst(RM_Dest_e dst);
void radio_set_msgtype(RM_Typ_e typ);
void radio_tx_set_message_header(RM_WhoAmI_e whoami, RM_Dest_e dst, RM_Typ_e typ);

/// ===================================== Radio message type =====================================

void radio_rx_get_tmp_id(uint16_t * tmp_id);
void radio_tx_set_tmp_id(uint16_t id);

uint8_t radio_rx_get_id();
void radio_tx_set_id(uint8_t id);

void radio_rx_get_radiodevinfo(RadioDevInfo *rdinfo);
void radio_tx_set_radiodevinfo(RadioDevInfo *rdinfo);

void radio_rx_get_sensordata(SensorData *snsrdata);
void radio_tx_set_sensordata(SensorData * sensordata);


/// ===================================== Sensor data type =====================================
void get_sensordata_type();


#endif //RADIO_DATA_FORMATS_H
