#ifndef INFRARED_H
#define INFRARED_H

#include "stm32f1xx_hal.h"

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

static inline void reset_transmitter();
static inline void switch_to_data_transmission_state();
static inline void p_w_modulate(uint8_t bit);
static inline void force_envelop_timer_output_on();
static inline void force_envelop_timer_output_off();


#endif //INFRARED_H
