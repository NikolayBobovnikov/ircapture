#ifndef MOCAP_DEV_H
#define MOCAP_DEV_H

#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_it.h"
#include "usbd_cdc_if.h"
#include <stdbool.h>

#include "common.h"
#include "radio_data_formats.h"
#include "se8r01.h"

void start_usb_device();
void register_usb_device();

#endif // MOCAP_DEV_H
