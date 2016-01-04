#ifndef INFRARED_H
#define INFRARED_H

#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"

#include <stdbool.h>
// TODO: cleanup when done debugging
#define DEBUG

/// Type definitions
// TODO: learn more about typedefs and structs
typedef struct
{
    uint8_t _1_beamer_id;
    uint8_t _2_angle_graycode;
    uint16_t _3_timer_cnt;
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
    STAGE_0,
    STAGE_OFF0,
    STAGE_OFF0_ON1,
    STAGE_ON1,
    STAGE_ON1_OFF1,
    STAGE_OFF1,
    STAGE_OFF1_ON2,
    STAGE_ON2,
    STAGE_ON2_OFF2,
    STAGE_OFF2,
    STAGE_OFF2_ON3,
    STAGE_ON3,
    STAGE_ON3_OFF4,
    STAGE_OFF4
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


/// Function prototypes

// main functions used in timer interrupt handlers
void irreceiver_timer_up_handler(); // for update timer
void irreceiver_timer_ic_handler(); // for input capture timer

// main routine called from timer interrupts to manage receiving process
void receive_handler();

// helper functions
bool is_1_to_0_edge();
bool is_0_to_1_edge();
bool is_1_on_update_event();
bool is_0_on_update_event();
void reset_receiver_state();
bool is_0_to_1_edge_timing_ok();
bool is_first_0_to_1_edge_timing_ok();
bool is_1_to_0_edge_timing_ok();
bool is_ic_after_interframe_delay();
void reset_delay_cnt();
void update_delay_cnt();

// function to copy data frame to the main buffer, when data is received successfully
void copy_data_frame_to_buffer(DataFrame_t* df);

// for debugging. TODO: cleanup when done
void dbg_pulse_1();
void dbg_pulse_2();

//#define DEBUG_READING_DATA_1
//#define DEBUG_READING_DATA_2
//#define DEBUG_DATA_RECEIVED_1
//#define DEBUG_DATA_RECEIVED_2
//#define DEBUG_UPD_EVENT_1
//#define DEBUG_UPD_EVENT_2
//#define DEBUG_LOW_CHECK_1
//#define DEBUG_LOW_CHECK_2
//#define DEBUG_DELAY_CHECK_1
//#define DEBUG_DELAY_CHECK_2
//#define DEBUG_DROP_DELAYCNT_1
//#define DEBUG_DROP_DELAYCNT_2
//#define DEBUG_0_to_1_EDGE_1
//#define DEBUG_0_to_1_EDGE_2
//#define DEBUG_1_to_0_EDGE_1
//#define DEBUG_1_to_0_EDGE_2


#endif //INFRARED_H
