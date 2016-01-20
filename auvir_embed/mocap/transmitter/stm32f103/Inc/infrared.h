#ifndef INFRARED_H
#define INFRARED_H

#include "stm32f1xx_hal.h"

// INFO: values below has been chosen manually
// need to work with IR receiver TL1838, and to be as low as possible,
// but not too low - beware of jitter!
// FIXME TODO: find mean and max for jitter  (about +- 30ns? need to check), and calculate minimum allowed values taken jitter into account
#define envelop_timer_prescaler     (72 - 1)     // values below are for particular prescaler
#define PreambleLongBitLength       (750 - 1)    // 270 works not reliably; 280 works;  chosen more
#define PreambleShortBitLength      (750 - 1)    // 270 works not reliably; 280 works;  chosen more
#define PreambleDelayLength         (500 - 1)    // 270 works not reliably; 280 works;  chosen more
#define DataBitLength               (500 - 1)    // TODO: Need to be distinguishable from start/stop bits. Start/Stop bit should on and off in less than data bit length
#define DelayBetweenDataFramesTotal (15000 - 1)  //12900 doesn't work; 13000 works; chosen more

#define pwm_timer_prescaler     0
#define pwm_timer_period        (1880 - 1)
#define pwm_pulse_width         (940 - 1)


enum TransmitterStates
{
    TX_WAITING,
    TX_START_BIT,
    TX_DATA,
    TX_STOP_BIT,
    TX_DELAY
};

enum DataFrameStates
{
    DATAFRAME_0_NODATA,
    DATAFRAME_1_BEAMER_ID,
    DATAFRAME_2_ANGLE,
    DATAFRAME_3_TIME
};

enum StartStopSequenceStates
{
    STAGE_0,
    STAGE_ON1,
    STAGE_OFF1,
    STAGE_ON2,
    STAGE_OFF2,
    STAGE_ON3,
    STAGE_OFF3
};

typedef struct
{
    uint8_t _1_beamer_id;
    uint8_t _2_angle_code;
    uint8_t _3_angle_code_rev;
} DataFrame_t;

void send_data();
void transmit_handler();

#endif //INFRARED_H
