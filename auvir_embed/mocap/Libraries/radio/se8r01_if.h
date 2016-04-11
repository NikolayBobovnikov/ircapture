#include "stm32f1xx_hal.h"
#include <stdbool.h>

#if 0
// description for bits in the message type byte
#define RM_SyncSensor       6 // 1 if sync signal for sensor, 0 otherwise
#define RM_SyncBeamer       5 // 1 if sync signal for beamer, 0 otherwise
#define RM_SendSensorData   4 // 1 if "is sensor data", 0 otherwise
#define RM_SendRegisterReq  3 // 1 if "is registration request", 0 otherwise
#define RM_WhoAmI_Sensor    2 // 1 for Sensor, 0 otherwise
#define RM_WhoAmI_Beamer    1 // 1 for Beamer, 0 otherwise
#define RM_WhoAmI_UsbDevice 0 // 1 if this is usbdevice, 0 otherwise (sensor/beamer)

#define is_usb_device(byte) ((byte) & (1 << RM_WhoAmI_UsbDevice))
#define is_beamer(byte)     ((byte) & (1 << RM_WhoAmI_Beamer) & (!(1 << RM_WhoAmI_UsbDevice)))
#define is_sensor(byte)     ((byte) & (1 << RM_WhoAmI_Sensor)&  (!(1 << RM_WhoAmI_UsbDevice)))
#define is_reg_request(byte)((byte) & (1 << RM_SendRegisterReq))
#define is_sensor_data(byte)((byte) & (1 << RM_SendSensorData))
#define is_sync_beamer(byte)((byte) & (1 << RM_SyncBeamer))
#define is_sync_sensor(byte)((byte) & (1 << RM_SyncSensor))
#endif

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
    Typ_RequestRegistration,
    Typ_RespondRegistration,
    Typ_SensorData,
    Typ_UpdateLastBeamerID,
    Typ_SyncBeamer,
    Typ_Reset
}RM_Typ_e;

// Bit      Description     Value
// 0:1      WhoAmI(Src)     0       UsbDevice   1 - Sensor  2 - Beamer  3 - <preserved>
// 2:3      Dst             0       UsbDevice   1 - Sensor  2 - Beamer  3 - <preserved>
// 4:6      Typ             0   000     RequestRegistration
//                          1   001     RespondRegistration
//                          2   010     SendData
//                          3   011     UpdateLastBeamerID
//                          4   100     SyncBeamer
//                          5   101     Reset
//                          6   110     <preserved>
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
    uint8_t data[30];
    const uint8_t check_zero;
}RadioMessage;

void nrf24_setup_modules_gpio();
