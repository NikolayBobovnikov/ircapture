#ifndef MOCAP_DEV_H
#define MOCAP_DEV_H

#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"
#include <stdbool.h>

#include "se8r01.h"
#include "common.h"

typedef enum UsbDeviceStates{
    USBState_None,
    USBState_Registration,
    USBState_ReceiveingData,
} UsbDeviceStates;

#endif //MOCAP_DEV_H
