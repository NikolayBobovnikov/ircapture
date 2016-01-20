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

// FIXME: TODO: keep values below in sync with transmitter
#define envelop_timer_prescaler     (72 - 1)     // values below are for particular prescaler
#define PreambleLongBitLength       (750 - 1)    // 270 works not reliably; 280 works;  chosen more
#define PreambleShortBitLength      (750 - 1)    // 270 works not reliably; 280 works;  chosen more
#define PreambleDelayLength         (500 - 1)    // 270 works not reliably; 280 works;  chosen more
#define DataBitLength               (500 - 1)    // TODO: Need to be distinguishable from start/stop bits. Start/Stop bit should on and off in less than data bit length
#define InterframeDelayLength       (15000 - 1)  //12900 doesn't work; 13000 works; chosen more
#define HalfDataBitLength   (250 - 1)   //(DataBitLength + 1) / 2 - 1; // TODO: check the value
#define max_delta_pwm_pulse 40          // 30 work unreliably, which means that error/drift variance is more than 30 ticks?. 35 works
#define max_delta_pwm_width 40          // 30 work unreliably, which means that error/drift variance is more than 30 ticks?. 35 works

#define PreambleProbingPeriod (10 - 1)
#define InterframeDelayProbingPeriod (100 - 1)

// take into account off-by-one offset in timer periods
#define InterframeDelayCounterExpected  ((InterframeDelayLength + 1 - PreambleDelayLength + 1) / InterframeDelayProbingPeriod) // TODO: check for necessity of PreambleDelayLength
#define PreambleLongBitCounterExpected  ((PreambleLongBitLength + 1)  / PreambleProbingPeriod)
#define PreambleShortBitCounterExpected ((PreambleShortBitLength + 1) / PreambleProbingPeriod)
#define PreambleDelayCounterExpected    ((PreambleDelayLength + 1)    / PreambleProbingPeriod)

#define max_delta_cnt_interframe_delay          (int)(InterframeDelayCounterExpected * 0.05) // 5%
#define max_delta_cnt_preamble_long_bit_length  (int)(PreambleLongBitCounterExpected * 0.1) // 10%
#define max_delta_cnt_preamble_short_bit_length (int)(PreambleShortBitCounterExpected * 0.1) // 10%
#define max_delta_cnt_preamble_delay_length     (int)(PreambleDelayCounterExpected * 0.1) // 10%


// take into account off-by-one offset in timer periods
#define InterframeDelayCounterMin   (InterframeDelayCounterExpected  - max_delta_cnt_interframe_delay)
#define PreambleLongBitCounterMin   (PreambleLongBitCounterExpected  - max_delta_cnt_preamble_long_bit_length)
#define PreambleShortBitCounterMin  (PreambleShortBitCounterExpected - max_delta_cnt_preamble_short_bit_length)
#define PreambleDelayCounterMin     (PreambleDelayCounterExpected    - max_delta_cnt_preamble_delay_length)


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
    STAGE_OFF0,
    STAGE_PREAMBLE_LONG_BIT,
    STAGE_PREAMBLE_DELAY_1,
    STAGE_PREAMBLE_SHORT_BIT,
    STAGE_PREAMBLE_DELAY_2
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
