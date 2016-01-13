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
static inline void receive_handler();

// helper functions
static inline bool is_1_to_0_edge();
static inline bool is_0_to_1_edge();
static inline bool is_1_on_update_event();
static inline bool is_0_on_update_event();
static inline void reset_receiver_state();
static inline bool is_0_to_1_edge_timing_ok();
static inline bool is_first_0_to_1_edge_timing_ok();
static inline bool is_1_to_0_edge_timing_ok();
static inline bool is_ic_after_interframe_delay();
static inline void reset_delay_cnt();
static inline void update_delay_cnt();
static inline void send_dataready_signal();

// function to copy data frame to the main buffer, when data is received successfully
static inline void copy_data_frame_to_buffer(DataFrame_t* df);

// for debugging. TODO: cleanup when done
static inline void dbg_pulse_1();
static inline void dbg_pulse_2();


#endif //INFRARED_H
