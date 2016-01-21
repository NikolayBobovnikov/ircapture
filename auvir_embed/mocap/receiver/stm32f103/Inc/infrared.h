#ifndef INFRARED_H
#define INFRARED_H

#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"

#include <stdbool.h>
// TODO: cleanup when done debugging
#define DEBUG

///====================== parameters ======================

#define pwm_timer_prescaler     0
#define pwm_timer_period        (1880 - 1)
#define pwm_pulse_width         (940 - 1)

//#define envelop_timer_prescaler     (72 - 1)     // values below are for particular prescaler
//#define PreambleBitLength           (750 - 1)    // 270 works not reliably; 280 works;  chosen more
//#define PreambleDelayLength         (750 - 1)    // 270 works not reliably; 280 works;  chosen more
//#define DataBitLength               (500 - 1)    // TODO: Need to be distinguishable from start/stop bits. Start/Stop bit should on and off in less than data bit length
//#define DelayBetweenDataFramesTotal (15000 - 1)  //12900 doesn't work; 13000 works; chosen more

// FIXME: TODO: keep values below in sync with transmitter
#define envelop_timer_prescaler     (72 - 1)     // values below are for particular prescaler
#define PreambleBitLength           (750 - 1)    // 270 works not reliably; 280 works;  chosen more
#define PreambleDelayLength         (750 - 1)    // 270 works not reliably; 280 works;  chosen more
#define DataBitLength               (500 - 1)    // TODO: Need to be distinguishable from start/stop bits. Start/Stop bit should on and off in less than data bit length
#define InterframeDelayLength       (15000 - 1)  //12900 doesn't work; 13000 works; chosen more
#define HalfDataBitLength   ((DataBitLength + 1) / 2 - 1)   //(DataBitLength + 1) / 2 - 1; // TODO: check the value
#define max_delta_pwm_pulse 40          // 30 work unreliably, which means that error/drift variance is more than 30 ticks?. 35 works
#define max_delta_pwm_width 40          // 30 work unreliably, which means that error/drift variance is more than 30 ticks?. 35 works
#define PreambleTotalLength         (3000 - 1)
#define max_period         (65535)

#define max_delta_interframe_delay  (int)(InterframeDelayLength * 0.05) // 10%
#define max_delta_preamble_bit      (int)(PreambleBitLength * 0.05) // 10%
#define max_delta_preamble_delay    (int)(PreambleDelayLength * 0.05) // 10%

///====================== Type definitions ======================
// TODO: learn more about typedefs and structs
typedef struct
{
    uint8_t _1_beamer_id;
    uint8_t _2_angle_code;
    uint8_t _3_angle_code_rev;
} DataFrame_t;
enum ReceiverStates
{
    RX_WAITING_FOR_START_BIT,
    RX_START_BIT_PROCESSING,
    RX_START_BIT_DONE,
    RX_DATA_PROCESSNG,
    RX_DATA_DONE,
    RX_STOP_BIT_PROCESSING,
    RX_STOP_BIT_DONE
};
enum StartStopSequenceStates
{
    STAGE_PREAMBLE_START,
    STAGE_PREAMBLE_BIT_1,
    STAGE_PREAMBLE_DELAY_1,
    STAGE_PREAMBLE_BIT_2,
    STAGE_PREAMBLE_DELAY_2,
    STAGE_PREAMBLE_STOP
};
enum DataFrameStates
{
    DATAFRAME_1_BEAMER_ID,
    DATAFRAME_2_ANGLE,
    DATAFRAME_3_TIME,
};
enum LineLevels
{
    LINE_UNDEFINED,
    LINE_LOW_ON_UPDATE_EVENT,
    LINE_HIGH_ON_UPDATE_EVENT
};


///====================== Function prototypes ======================
// main functions used in timer interrupt handlers
void irreceiver_timer_prob_handler(); // for update timer
void irreceiver_timer_up_handler(); // for update timer
void irreceiver_timer_ic_handler(); // for input capture timer


#endif //INFRARED_H
