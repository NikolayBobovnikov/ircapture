#ifndef MOCAP_DEV_H
#define MOCAP_DEV_H

#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"
#include "usbd_cdc_if.h"
#include <stdbool.h>

#include "se8r01.h"
#include "se8r01_if.h"
#include "common.h"

typedef struct USB_Message USB_Message;

void register_usb_device();

#endif //MOCAP_DEV_H
