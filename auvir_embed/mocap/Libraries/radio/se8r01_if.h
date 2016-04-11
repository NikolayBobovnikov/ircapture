#include "stm32f1xx_hal.h"
#include <stdbool.h>

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

/// Notes:
/// Typ_Reset can be sent only from a usbdevice
/// RM_WhoAmI_e cannot be equal to RM_Dest_e
/// RM_WhoAmI_e probably not needed, RM_Dest_e is enough:
/// To Beamer/Sensor from only usbdevice; To Usbdevice from only Beamer/sensor
/// Redundancy is probably not bad, can do additional check


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
RM_WhoAmI_e radio_get_whoami(uint8_t byte);
RM_Dest_e radio_get_dst(uint8_t byte);
RM_Typ_e radio_get_msgtype(uint8_t byte);
void radio_set_whoami(uint8_t * byte, RM_WhoAmI_e whoami);
void radio_set_dst(uint8_t * byte, RM_Dest_e dst);
void radio_set_msgtype(uint8_t * byte, RM_Typ_e typ);


// typedef struct RadioMessage RadioMessage; TODO
typedef struct RadioMessage{
    uint8_t type;
    uint8_t data[31];
}RadioMessage;

//Contents of data depending of the message type:
//Typ_BeamerRequestRegistration
//Typ_SensorRequestRegistration
//Typ_RespondRegistration
//Typ_SensorData
//Typ_UpdateLastBeamerID
//Typ_SyncBeamer
//Typ_Reset

void nrf24_setup_modules_gpio();
