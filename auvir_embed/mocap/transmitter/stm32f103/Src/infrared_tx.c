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
extern const uint32_t pwm_tim_channel;

TxStartStopSequenceStates StartStopSequenceTransmitState = Tx_PREAMBLE_BIT_1;
DataFrameStates TxDataFrameState = DATAFRAME_0_NODATA;
TransmitterStates TransmitterState = TX_WAITING;
DataFrame_t tx_data_frame;

uint8_t tx_total_bits = 0;
uint8_t tx_current_bit_pos = 0;
uint8_t tx_bit = 0;

uint8_t beamer_ir_channel_array[NUMBER_OF_BEAMER_CHANNELS + 1] =
{
  0b00000001,//0
  0b00000010,//1
  0b00000100,//2
  0b00001000,//3
  0b00010000,//4
  0b00100000,//5
  0b01000000,//6
  0b10000000 //7
  //0b0000001000000000,//9
  //0b0000010000000000,//10 data pin
};
uint8_t beamer_current_ir_channel = 0;
const uint8_t beamer_data_pin = NUMBER_OF_BEAMER_CHANNELS;
const uint8_t beamer_ir_1st_channel = 0;

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
    // phtim_pwm generates PWM

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
                    beamer_current_ir_channel = beamer_data_pin;
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
    beamer_current_ir_channel = beamer_data_pin;
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

    shiftreg_send_8bit_data(0);
}

static inline void select_next_beamer_channel_index()
{
    if(beamer_current_ir_channel < NUMBER_OF_BEAMER_CHANNELS - 1){
        beamer_current_ir_channel++;
    }
    else{
        beamer_current_ir_channel = 0;
    }
}

static inline void reset_previous_update_current_beamer_pin()
{
    //TODO: IR channel turn off current pin?
    //shiftreg_send_16bit_data(0xFF ^ beamer_ir_channel_array[beamer_current_ir_channel]);
    shiftreg_send_8bit_data(0);

    if(DATAFRAME_2_ANGLE == TxDataFrameState){
        //TODO: IR channel - set current pin from IR channels
        beamer_current_ir_channel = beamer_ir_1st_channel;
    }
    else{
        beamer_current_ir_channel = beamer_data_pin;
    }
}

static inline void force_envelop_timer_output_on()
{
    // turn on current pin
    shiftreg_send_8bit_data(beamer_ir_channel_array[beamer_current_ir_channel]);

    // turn on carrier
    if(_is_direct_logic)
    {
        HAL_TIM_PWM_Start(phtim_pwm, pwm_tim_channel);
    }
    else
    {
        HAL_TIM_PWM_Stop(phtim_pwm, pwm_tim_channel);
    }

}

static inline void force_envelop_timer_output_off()
{
    // turn on current pin
    //TODO: IR channel turn on current pin [ why turn on? ]
    shiftreg_send_8bit_data(beamer_ir_channel_array[beamer_current_ir_channel]);

    // turn off carrier
    if(_is_direct_logic)
    {
        HAL_TIM_PWM_Stop(phtim_pwm, pwm_tim_channel);
    }
    else
    {
        HAL_TIM_PWM_Start(phtim_pwm, pwm_tim_channel);
    }

}
