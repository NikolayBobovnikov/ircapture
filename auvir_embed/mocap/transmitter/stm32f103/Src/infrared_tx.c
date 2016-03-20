#include "infrared.h"
#include <stdbool.h>

/// ================== Parameters ================
//TODO: cleanup when done debugging
extern const bool _debug;
extern const bool _is_direct_logic;

const bool _is_direct_logic = true;

//PWM timer configuration
extern TIM_HandleTypeDef * phtim_envelop;
extern TIM_HandleTypeDef * phtim_pwm;

TxStartStopSequenceStates StartStopSequenceTransmitState = Tx_PREAMBLE_BIT_1;
DataFrameStates TxDataFrameState = DATAFRAME_0_NODATA;
TransmitterStates TransmitterState = TX_WAITING;
DataFrame_t tx_data_frame;

uint8_t tx_total_bits = 0;
uint8_t tx_current_bit_pos = 0;
uint8_t tx_bit = 0;

// constant, use for transmission non-structured light data
const MCU_PIN standard_data_pin = { GPIOB, GPIO_PIN_12};
// variable, use for transmission coded angle
MCU_PIN current_pin  = { GPIOB, GPIO_PIN_12};

MCU_PIN beamer_channel_array[NUMBER_OF_BEAMER_CHANNELS] = {
    { GPIOB, GPIO_PIN_0},
    { GPIOB, GPIO_PIN_1},
    { GPIOB, GPIO_PIN_3},
    { GPIOB, GPIO_PIN_4},
    { GPIOB, GPIO_PIN_5},
    { GPIOB, GPIO_PIN_6},
    { GPIOB, GPIO_PIN_7},
    { GPIOB, GPIO_PIN_8}
};
uint8_t current_beamer_channel_index = 0;


/// ================== External Function prototypes ================
void RXX();
void TXX();

/// ============================== Private function declarations ==============================
static inline void reset_transmitter();
static inline void switch_to_data_transmission_state();
static inline void p_w_modulate(uint8_t bit);
static inline void turn_off_all_beamer_pins();
static inline void select_next_beamer_channel_index();
static inline void reset_previous_update_current_beamer_pin();
static inline void force_envelop_timer_output_on();
static inline void force_envelop_timer_output_off();

/// ============================== Function definitions ==============================

// TX
void init_beamer_channels_gpio()
{
    GPIO_InitTypeDef GPIO_InitStruct;
    // initialize separate pins
    for(uint8_t pin_index = 0; pin_index < NUMBER_OF_BEAMER_CHANNELS; pin_index++)
    {
        GPIO_InitStruct.Pin = beamer_channel_array[pin_index].pin_number;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

        HAL_GPIO_Init(beamer_channel_array[pin_index].pin_port, &GPIO_InitStruct);
    }

    // initialize common pin
    GPIO_InitStruct.Pin = standard_data_pin.pin_number;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(standard_data_pin.pin_port, &GPIO_InitStruct);
}

void init_data()
{
    // sample data. TODO: use actual one
    tx_data_frame._1_beamer_id = 0b10101010;
    tx_data_frame._2_angle_code = 0b11111111;
    tx_data_frame._3_angle_code_rev = ~(tx_data_frame._2_angle_code);

    // just repetition of the same data for now.
    // TODO: send updated time
    if(TX_WAITING == TransmitterState)
    {
        TransmitterState = TX_PREAMBLE;
    }
    else
    {
        //TODO: handle this case
        // data is still being transmitted. Need to finish previous transmission before starting next one
    }
}

void sensor_send_data()
{
    // TODO: find suitable place for this
    //tx_data_frame._3_angle_code_rev = ~(tx_data_frame._2_angle_code);

    if(TX_WAITING == TransmitterState)
    {
        TransmitterState = TX_PREAMBLE;
    }
    else
    {
        //TODO: handle this case
        // data is still being transmitted. Need to finish previous transmission before starting next one
    }
}

void transmit_handler()
{
    /* Data packet format: [preamble] [data frame] [epilogue]
     * preamble format: [longbit] [delay1] [bit1] [delay2]
     * data frame format: [word1] [delay] [word2] [delay] [word3]
     * data frame format: [delay1] [bit1] [delay2] [longbit]
     *
     *                      |<--     Preamble          -->|<--              data frame              -->|<--        epilogue  -->|
     *                       ______________      ____      ________          ________          ________      ____      _____________
     *                      |              |    |    |    |        |        |        |        |        |    |    |    |             |
     *                      |              |    |    |    |dataword|delay   |        |        |        |    |    |    |             |
     *                      |     750      |350 |350 |350 |  500   |   500  |        |        |  500   |350 |350 |350 |    750      |
     *  ____________________|              |____|    |____|        |________|        |__....__|        |____|    |____|             |____
     *
     *
     *  |<----------------->| DelayBetweenDataFramesTotal
     *
     *                      |<-->|<-->| StartStopBitLength
     *                                                     |<------>|<------>| DataBitLength
     *
     *
     *
     * 1. start sequence
     * 2. data
     *  2.1 Coded angle
     *  2.2 Coded angle reversed (to verify correctness)
     *  2.3 Beamer ID
     * 3. data redundancy (repeated, or use error correction code)
     * 4. stop sequence
     *
     * Check data integrity. If OK, data is received
     */
    /// ensure carrier is not generating
    if(TX_DATA != TransmitterState)
    {
        force_envelop_timer_output_off();
        turn_off_all_beamer_pins();
    }
    /* Send data frame
     * 1. start sequence
     * 2. data
     *  2.1 Coded angle
     *  2.2 Time of emission
     *  2.3 Beamer ID
     * 3. data redundancy (repeated, or use error correction code)
     * 4. stop sequence
     */
    // htim_envelop forming envelop
    // htim2 generates PWM

    switch(TransmitterState)
    {
        case TX_WAITING:
        {
            ; // do nothing
            break;
        }
        case TX_PREAMBLE:
        {
            // Start sequence consists of signal sequence {1,0,1}
            switch(StartStopSequenceTransmitState)
            {
                // First (long) bit
                case Tx_PREAMBLE_BIT_1:
                {
                    force_envelop_timer_output_on();
                    phtim_envelop->Instance->ARR = PreambleBitCorrected;
                    StartStopSequenceTransmitState = Tx_PREAMBLE_DELAY_1;
                    break;
                }
                // First short delay
                case Tx_PREAMBLE_DELAY_1:
                {
                    force_envelop_timer_output_off(); //TODO done anyway in timer interrupt handler?
                    phtim_envelop->Instance->ARR = PreambleDelayCorrected;
                    StartStopSequenceTransmitState = Tx_PREAMBLE_BIT_2;
                    break;
                }
                // Second (short) bit
                case Tx_PREAMBLE_BIT_2:
                {
                    force_envelop_timer_output_on();
                    phtim_envelop->Instance->ARR = PreambleBitCorrected;
                    StartStopSequenceTransmitState = Tx_PREAMBLE_DELAY_2;
                    break;
                }
                // Second short delay
                case Tx_PREAMBLE_DELAY_2:
                {
                    force_envelop_timer_output_off();
                    phtim_envelop->Instance->ARR = PreambleDelayCorrected;

                    StartStopSequenceTransmitState = Tx_PREAMBLE_BIT_1;
                    TransmitterState = TX_DATA;
                    TxDataFrameState = DATAFRAME_1_BEAMER_ID;
                    tx_current_bit_pos = 0;
                    break;
                }
            }
            break;
        }
        case TX_DATA:
        {
            phtim_envelop->Instance->ARR = DataBitLength;

            switch (TxDataFrameState)
            {
                case(DATAFRAME_1_BEAMER_ID):
                {
                    tx_total_bits = sizeof(tx_data_frame._1_beamer_id) * 8;
                    tx_bit = (tx_data_frame._1_beamer_id >> (tx_total_bits - tx_current_bit_pos - 1)) & 1;
                    p_w_modulate(tx_bit);
                    // go to next bit.
                    if(tx_current_bit_pos < tx_total_bits)
                    {
                        tx_current_bit_pos++;
                    }
                    else
                    {
                        // change state to process next part of data
                        TxDataFrameState = DATAFRAME_2_ANGLE;
                        tx_current_bit_pos = 0;
                    }
                    break;
                }
                case(DATAFRAME_2_ANGLE):
                {
                    reset_previous_update_current_beamer_pin();

                    tx_total_bits = sizeof(tx_data_frame._2_angle_code) * 8;
                    tx_bit = (tx_data_frame._2_angle_code >> (tx_total_bits - tx_current_bit_pos - 1)) & 1;
                    p_w_modulate(tx_bit);
                    // go to next bit.
                    if(tx_current_bit_pos < tx_total_bits)
                    {
                        tx_current_bit_pos++;
                    }
                    else
                    {
                        // change state to process next part of data
                        TxDataFrameState = DATAFRAME_3_ANGLE_REV;
                        tx_current_bit_pos = 0;
                    }

                    select_next_beamer_channel_index();

                    break;
                }
                case(DATAFRAME_3_ANGLE_REV):
                {
                    tx_total_bits = sizeof(tx_data_frame._3_angle_code_rev) * 8;
                    tx_bit = (tx_data_frame._3_angle_code_rev >> (tx_total_bits - tx_current_bit_pos - 1)) & 1;
                    p_w_modulate(tx_bit);
                    // go to next bit.
                    if(tx_current_bit_pos < tx_total_bits)
                    {
                        tx_current_bit_pos++;
                    }
                    else
                    {
                        /// start epologue

                        /* Set the Autoreload value for start sequence bits*/
                        force_envelop_timer_output_off();
                        phtim_envelop->Instance->ARR = EpilogueDelayCorrected;

                        // move on to next stage
                        TransmitterState = TX_EPILOGUE;
                        StartStopSequenceTransmitState = Tx_PREAMBLE_BIT_1;
                        TxDataFrameState = DATAFRAME_0_NODATA;
                        tx_current_bit_pos = 0;
                    }

                    select_next_beamer_channel_index();

                    break;
                }
                default:
                {
                    reset_transmitter();
                    return;
                }
            }

            // TODO: check if some errors or other options are possible here?
            break;
        }
        case TX_EPILOGUE:
        {
            reset_previous_update_current_beamer_pin();

            // Start sequence consists of signal sequence {1,0,1}
            switch(StartStopSequenceTransmitState)
            {
                case Tx_PREAMBLE_BIT_1:
                {
                    force_envelop_timer_output_on();
                    phtim_envelop->Instance->ARR = EpilogueBitCorrected;
                    StartStopSequenceTransmitState = Tx_PREAMBLE_DELAY_1;
                    break;
                }
                case Tx_PREAMBLE_DELAY_1:
                {
                    reset_transmitter();
                    break;
                }
                default:
                {
                    reset_transmitter();
                    break;
                }
            }
            break;
        }
        default:
        {
            reset_transmitter();
            break;
        }
    } // switch(TransmitterState)
}

static inline void reset_transmitter()
{
    force_envelop_timer_output_off();
    TransmitterState = TX_WAITING;
    phtim_envelop->Instance->ARR = InterframeDelayLength;
    StartStopSequenceTransmitState = Tx_PREAMBLE_BIT_1;
    TxDataFrameState = DATAFRAME_0_NODATA;
    current_pin = standard_data_pin;
    current_beamer_channel_index = 0;
}

static inline void switch_to_data_transmission_state()
{

}

static inline void p_w_modulate(uint8_t bit)
{
    // modulate logic 0 and 1
    if(bit == 1){
        force_envelop_timer_output_on();
    }
    else{
        force_envelop_timer_output_off();
    }
}

static inline void turn_off_all_beamer_pins()
{

    for(uint8_t pin_index = 0; pin_index < NUMBER_OF_BEAMER_CHANNELS; pin_index++)
    {
        HAL_GPIO_WritePin(beamer_channel_array[pin_index].pin_port, beamer_channel_array[pin_index].pin_number, GPIO_PIN_RESET);
    }

    HAL_GPIO_WritePin(standard_data_pin.pin_port, standard_data_pin.pin_number,  GPIO_PIN_RESET);
}

static inline void select_next_beamer_channel_index()
{
    if(current_beamer_channel_index < NUMBER_OF_BEAMER_CHANNELS - 1){
        current_beamer_channel_index++;
    }
    else{
        current_beamer_channel_index = 0;
    }
}

static inline void reset_previous_update_current_beamer_pin()
{
    HAL_GPIO_WritePin(current_pin.pin_port, current_pin.pin_number, GPIO_PIN_RESET);

    if(DATAFRAME_2_ANGLE == TxDataFrameState){
        current_pin = beamer_channel_array[current_beamer_channel_index];
    }
    else{
        current_pin = standard_data_pin;
    }
}

static inline void force_envelop_timer_output_on()
{
    if(_debug)
    {
        if(_is_direct_logic)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
        }
    }

    // turn on current pin
    HAL_GPIO_WritePin(current_pin.pin_port,current_pin.pin_number, GPIO_PIN_SET);

    // turn on carrier
    if(_is_direct_logic)
    {
        HAL_TIM_PWM_Start(phtim_pwm, TIM_CHANNEL_4);
    }
    else
    {
        HAL_TIM_PWM_Stop(phtim_pwm, TIM_CHANNEL_4);
    }

}

static inline void force_envelop_timer_output_off()
{
    if(_debug)
    {
        if(_is_direct_logic)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
        }
    }

    // turn on current pin
    HAL_GPIO_WritePin(current_pin.pin_port,current_pin.pin_number, GPIO_PIN_SET);

    // turn off carrier
    if(_is_direct_logic)
    {
        HAL_TIM_PWM_Stop(phtim_pwm, TIM_CHANNEL_4);
    }
    else
    {
        HAL_TIM_PWM_Start(phtim_pwm, TIM_CHANNEL_4);
    }

}
