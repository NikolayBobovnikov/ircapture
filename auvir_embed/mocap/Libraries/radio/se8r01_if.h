#include "stm32f1xx_hal.h"
#include <stdbool.h>

// description for bits in the message type byte
#define RM_SyncSensor       6 // 1 if sync signal for sensor, 0 otherwise
#define RM_SyncBeamer       5 // 1 if sync signal for beamer, 0 otherwise
#define RM_SendSensorData   4 // 1 if "is sensor data", 0 otherwise
#define RM_SendRegisterReq  3 // 1 if "is registration request", 0 otherwise
#define RM_WhoAmI_Sensor    2 // 1 for Sensor, 0 otherwise
#define RM_WhoAmI_Beamer    1 // 1 for Beamer, 0 otherwise
#define RM_WhoAmI_UsbDevice 0 // 1 if this is usbdevice, 0 otherwise

#define is_usb_device(byte) ((byte) & (1 << RM_WhoAmI_UsbDevice))
#define is_beamer(byte)     ((byte) & (1 << RM_WhoAmI_Beamer))
#define is_sensor(byte)     ((byte) & (1 << RM_WhoAmI_Sensor))
#define is_reg_request(byte)((byte) & (1 << RM_SendRegisterReq))
#define is_sensor_data(byte)((byte) & (1 << RM_SendSensorData))
#define is_sync_beamer(byte)((byte) & (1 << RM_SyncBeamer))
#define is_sync_sensor(byte)((byte) & (1 << RM_SyncSensor))

// typedef struct RadioMessage RadioMessage; TODO
typedef struct RadioMessage{
    uint8_t type;
    uint8_t data[30];
    const uint8_t check_zero;
}RadioMessage;

void nrf24_setup_modules_gpio();
