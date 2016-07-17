#ifndef INFRARED_H
#define INFRARED_H

#include "stm32f1xx_hal.h"
#include "common.h"
#include <stdbool.h>

// INFO: values below has been chosen manually
// need to work with IR receiver TL1838, and to be as low as possible,
// but not too low - beware of jitter!
// FIXME TODO: find mean and max for jitter  (about +- 30ns? need to check), and calculate minimum allowed values taken jitter into account
#define envelop_timer_prescaler     (72 - 1)     // values below are for particular prescaler
#define PreambleBitLength           (750 - 1)    // 270 works not reliably; 280 works;  chosen more
#define PreambleDelayLength         (750 - 1)    // 270 works not reliably; 280 works;  chosen more.
#define DataBitLength               (500 - 1)    // TODO: Need to be distinguishable from start/stop bits. Start/Stop bit should on and off in less than data bit length
#define InterframeDelayLength       (15000 - 1)  //12900 doesn't work; 13000 works; chosen more

#define DelayCheckingPeriod     (100 - 1)

// TODO: calculate mean of delay/bit length, deviation, and find optimal shift to minimize deviation
#define PreambleDelayShift      20//(30)
#define PreambleBitShift        10//(20)
#define EpilogueDelayShift      10//(20)
#define EpilogueBitShift        0//(10)
#define PreambleDelayCorrected  (PreambleDelayLength + PreambleDelayShift - 1)
#define PreambleBitCorrected    (PreambleBitLength - PreambleBitShift - 1)
#define EpilogueDelayCorrected  (PreambleDelayLength + EpilogueDelayShift - 1)
#define EpilogueBitCorrected    (PreambleBitLength - EpilogueBitShift - 1)
#define pwm_timer_prescaler     0
#define pwm_timer_period        (1880 - 1)
#define pwm_pulse_width         (940 - 1)

#define PreambleTotalLength       (PreambleBitLength * 2 + PreambleDelayLength * 2)
#define HalfDataBitLength   		((DataBitLength + 1) / 2 - 1)   //(DataBitLength + 1) / 2 - 1; // TODO: check the value
#define max_period                (65535 - 1)

#define max_delta_interframe_delay  (int)(InterframeDelayLength * 0.1)
#define max_delta_preamble_bit      (int)(PreambleBitLength * 0.1)
#define max_delta_preamble_delay    (int)(PreambleDelayLength * 0.1)

#define NUMBER_OF_BEAMER_CHANNELS 8 //8 ir channels, no data pin included
#define RX_BUF_SIZE 10

//======= Turn on/off particular testing pulses
#define DEBUG   0

#define DEBUG_FRAME_DELAY_1     0
#define DEBUG_FRAME_DELAY_2     0

#define DEBUG_UPD_EVENT_1       0
#define DEBUG_UPD_EVENT_2       0
#define DEBUG_0_to_1_EDGE_1     0
#define DEBUG_0_to_1_EDGE_2     0
#define DEBUG_1_to_0_EDGE_1     0
#define DEBUG_1_to_0_EDGE_2     0

#define DEBUG_PREAMBLE_END_1    0
#define DEBUG_PREAMBLE_END_2    0

#define DEBUG_EPILOGUE_BEGIN_1  0
#define DEBUG_EPILOGUE_BEGIN_2  0
#define DEBUG_EPILOGUE_END_1    0
#define DEBUG_EPILOGUE_END_2    0

#define DEBUG_READING_DATA_1    1
#define DEBUG_READING_DATA_2    0
#define DEBUG_DATA_VERIFIED_1   0
#define DEBUG_DATA_VERIFIED_2   1
#define DEBUG_DATA_END_1        0
#define DEBUG_DATA_END_2        0
#define DEBUG_DATA_RECEIVED_1   0
#define DEBUG_DATA_RECEIVED_2   0

typedef enum TransmitterStates
{
    TX_WAITING,
    TX_PREAMBLE,
    TX_DATA,
    TX_EPILOGUE
} TransmitterStates;

// TODO: unite
typedef enum DataFrameStates
{
    DATAFRAME_0_NODATA,
    DATAFRAME_1_BEAMER_ID,
    DATAFRAME_2_ANGLE,
    DATAFRAME_3_ANGLE_REV
} DataFrameStates;

typedef enum TxStartStopSequenceStates
{
    Tx_PREAMBLE_BIT_1,
    Tx_PREAMBLE_DELAY_1,
    Tx_PREAMBLE_BIT_2,
    Tx_PREAMBLE_DELAY_2
} TxStartStopSequenceStates;

typedef enum RxStartStopSequenceStates
{
    Rx_PREAMBLE_START,
    Rx_PREAMBLE_BIT_1,
    Rx_PREAMBLE_DELAY_1,
    Rx_PREAMBLE_BIT_2,
    Rx_PREAMBLE_DELAY_2,
    Rx_PREAMBLE_STOP
} RxStartStopSequenceStates;

typedef struct DataFrame_t
{
    uint8_t _1_beamer_id;
    uint8_t _2_angle_code;
    uint8_t _3_angle_code_rev;
} DataFrame_t;

typedef enum ReceiverStates
{
    RX_WAITING_FOR_START_BIT,
    RX_START_BIT_PROCESSING,
    RX_START_BIT_DONE,
    RX_DATA_PROCESSNG,
    RX_DATA_DONE,
    RX_STOP_BIT_PROCESSING,
    RX_STOP_BIT_DONE
} ReceiverStates;

typedef enum LineLevels
{
    LINE_UNDEFINED,
    LINE_LOW_ON_UPDATE_EVENT,
    LINE_HIGH_ON_UPDATE_EVENT
} LineLevels;

typedef struct MCU_PIN{
    GPIO_TypeDef * pin_port;
    uint16_t pin_number;
} MCU_PIN;

///====================== Function prototypes ======================
void init_data();
void sensor_send_data();
void transmit_handler();

// main functions used in timer interrupt handlers
void irreceiver_timer_prob_handler(); // for update timer
void irreceiver_timer_up_handler(); // for update timer
void irreceiver_timer_ic_handler(); // for input capture timer
void setup_ic_timer();

#endif //INFRARED_H
