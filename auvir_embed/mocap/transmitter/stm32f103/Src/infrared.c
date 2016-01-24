#include "infrared.h"
#include <stdbool.h>


/// ================== Parameters ================
//TODO: cleanup when done debugging
extern const bool _debug;
//TODO: cleanup when done debugging
const bool _is_direct_logic = true;


//PWM timer configuration
extern TIM_HandleTypeDef * phtim_envelop;
extern TIM_HandleTypeDef * phtim_pwm;

/// ================== Variables ================
uint8_t StartStopSequenceTransmitState = STAGE_PREAMBLE_BIT_1;
uint8_t DataFrameState = DATAFRAME_0_NODATA;
uint8_t TransmitterState = TX_WAITING;

///TODO: refactor constants below
DataFrame_t tx_data_frame;
volatile size_t tx_total_bits = 0;
volatile uint8_t tx_current_bit_pos = 0;
volatile uint8_t tx_bit = 0;

uint8_t level[100] = {0};
uint16_t pwm_period[100] = {0};
uint8_t arr_index = 0;


/// ============================== Function declarations ==============================
static inline void reset_transmitter();
static inline void switch_to_data_transmission_state();
static inline void p_w_modulate(uint8_t bit);
static inline void force_envelop_timer_output_on();
static inline void force_envelop_timer_output_off();


/// ============================== Function definitions ==============================
///
void init_data()
{
    // sample data. TODO: use actual one
    tx_data_frame._1_beamer_id = 0b10101010;
    tx_data_frame._2_angle_code = 0b10101010;
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
void send_data()
{
    // TODO: find suitable place for this
    tx_data_frame._3_angle_code_rev = ~(tx_data_frame._2_angle_code);

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
        force_envelop_timer_output_off();  // stop carrier // TODO: check neseccity
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
                case STAGE_PREAMBLE_BIT_1:
                {
                    force_envelop_timer_output_on();
                    phtim_envelop->Instance->ARR = PreambleBitLength;
                    StartStopSequenceTransmitState = STAGE_PREAMBLE_DELAY_1;
                    break;
                }
                // First short delay
                case STAGE_PREAMBLE_DELAY_1:
                {
                    force_envelop_timer_output_off(); //TODO done anyway in timer interrupt handler?
                    phtim_envelop->Instance->ARR = PreambleDelayLength;
                    StartStopSequenceTransmitState = STAGE_PREAMBLE_BIT_2;
                    break;
                }
                // Second (short) bit
                case STAGE_PREAMBLE_BIT_2:
                {
                    force_envelop_timer_output_on();
                    phtim_envelop->Instance->ARR = PreambleBitLength;
                    StartStopSequenceTransmitState = STAGE_PREAMBLE_DELAY_2;
                    break;
                }
                // Second short delay
                case STAGE_PREAMBLE_DELAY_2:
                {
                    force_envelop_timer_output_off();
                    phtim_envelop->Instance->ARR = PreambleDelayLength;

                    StartStopSequenceTransmitState = STAGE_PREAMBLE_BIT_1;
                    TransmitterState = TX_DATA;
                    DataFrameState = DATAFRAME_1_BEAMER_ID;
                    tx_current_bit_pos = 0;
                    break;
                }
            }
            break;
        }
        case TX_DATA:
        {
            phtim_envelop->Instance->ARR = DataBitLength;

            switch (DataFrameState)
            {
                default:
                {
                    reset_transmitter();
                    return;
                }
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
                        DataFrameState = DATAFRAME_2_ANGLE;
                        tx_current_bit_pos = 0;
                    }
                    break;
                }
                case(DATAFRAME_2_ANGLE):
                {
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
                        DataFrameState = DATAFRAME_3_TIME;
                        tx_current_bit_pos = 0;
                    }
                    break;
                }
                case(DATAFRAME_3_TIME):
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
                        phtim_envelop->Instance->ARR = PreambleDelayLength;

                        // move on to next stage
                        TransmitterState = TX_EPILOGUE;
                        StartStopSequenceTransmitState = STAGE_PREAMBLE_BIT_1;
                        DataFrameState = DATAFRAME_0_NODATA;
                        tx_current_bit_pos = 0;
                    }
                    break;
                }
            }

            // TODO: check if some errors or other options are possible here?
            break;
        }
        case TX_EPILOGUE:
        {
            // Start sequence consists of signal sequence {1,0,1}
            switch(StartStopSequenceTransmitState)
            {
                case STAGE_PREAMBLE_BIT_1:
                {
                    force_envelop_timer_output_on();
                    phtim_envelop->Instance->ARR = PreambleBitLength;
                    StartStopSequenceTransmitState = STAGE_PREAMBLE_DELAY_1;
                    break;
                }
                case STAGE_PREAMBLE_DELAY_1:
                {
                    reset_transmitter();
                    break;
                }
            }
            break;
        }
    } // switch(TransmitterState)
}
/// private, used only in infrared module
static inline void reset_transmitter()
{
    force_envelop_timer_output_off();
    TransmitterState = TX_WAITING;
    phtim_envelop->Instance->ARR = DelayBetweenDataFramesTotal;
    StartStopSequenceTransmitState = STAGE_PREAMBLE_BIT_1;
    DataFrameState = DATAFRAME_0_NODATA;
}
static inline void switch_to_data_transmission_state()
{

}
static inline void p_w_modulate(uint8_t bit)
{
    if(bit == 1)
    {
        force_envelop_timer_output_on();
    }
    else // bit == 0
    {
        force_envelop_timer_output_off();
    }

}
static inline void force_envelop_timer_output_on()
{
    if(_debug)
    {
        if(_is_direct_logic)
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
        }
    }
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
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
        }
    }

    if(_is_direct_logic)
    {
        HAL_TIM_PWM_Stop(phtim_pwm, TIM_CHANNEL_4);
    }
    else
    {
        HAL_TIM_PWM_Start(phtim_pwm, TIM_CHANNEL_4);
    }

}
