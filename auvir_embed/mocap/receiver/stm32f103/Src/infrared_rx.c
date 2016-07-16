#include "infrared.h"
#include <stdbool.h>

/// ================== Parameters ================

extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef* ptim_input_capture;
extern TIM_HandleTypeDef* ptim_data_read;

extern GPIO_TypeDef * GPIO_PORT_IR_IN;
extern uint16_t GPIO_PIN_IR_IN;
extern const bool _is_direct_logic;

const bool _is_direct_logic = false;

DataFrame_t data_frames[RX_BUF_SIZE] = {0}; // TODO: verify initialization
uint8_t arr_index = 0;

DataFrame_t rx_data_frame;
uint8_t data_frame_delta = 0;
RxStartStopSequenceStates ReceiverState = RX_WAITING_FOR_START_BIT;
RxStartStopSequenceStates StartStopSequenceReceiveState = Rx_PREAMBLE_START;
DataFrameStates DataFrameState = DATAFRAME_1_BEAMER_ID;
LineLevels LineLevelState = LINE_UNDEFINED;

uint8_t rx_data = 0;
uint8_t rx_total_bits = 0;
uint8_t rx_current_bit_pos = 0;
uint8_t rx_bit = 0;

bool _is_rising_edge = false;
bool _is_falling_edge = false;
bool _is_uptimer_update_event = false;

uint16_t _ccr1 = 0;
uint16_t _ccr2 = 0;

GPIO_TypeDef * GPIO_LED_PORT = GPIOB;
uint16_t GPIO_LED_PIN = GPIO_PIN_3;


/// ================== Radio interface ================

extern volatile uint8_t rx_buf[32]; // initialize value
extern volatile uint8_t tx_buf[32];

void RXX();
void TXX();

/// ============================== Private function declarations ==============================

//receiver functions
static inline void receive_handler();
static inline bool is_1_to_0_edge();
static inline bool is_0_to_1_edge();
static inline bool is_1_on_update_event();
static inline bool is_0_on_update_event();
static inline void reset_receiver_state();
static inline void send_dataready_signal();
static inline void get_logical_level();
static inline void decode_bit(uint8_t *data_word);
static inline void process_received_data();
static inline void copy_data_frame_to_buffer(DataFrame_t* df);
void send_data_uart();
static inline bool is_correct_timming_interframe_delay();
static inline bool is_correct_timming_preamble_bit();
static inline bool is_correct_timming_preamble_delay();

// for debugging. TODO: cleanup when done
static inline void dbg_pulse_1();
static inline void dbg_pulse_2();
static inline void debug_interframe_delay();
static inline void debug_reading_data();
static inline void debug_data_verified();
static inline void debug_data_end();
static inline void debug_data_received();
static inline void debug_upd_event();
static inline void debug_0_to_1_edge();
static inline void debug_1_to_0_edge();
static inline void debug_epilogue_begin();
static inline void debug_epilogue_end();
static inline void debug_preamble_end();


/// ============================== Function definitions ==============================

#define DECODE_BIT_IN_WORD(word) ( word |= 1 << (rx_total_bits - rx_current_bit_pos - 1) )

static inline void decode_bit(uint8_t *data_word) {
    if(is_1_on_update_event())
    {
        // 1) k-th bit of n: (n >> k) & 1
        // 2) set bit at the inversed position
        DECODE_BIT_IN_WORD( *data_word );
    }
}

inline void irreceiver_timer_up_handler()
{
    debug_upd_event();

    get_logical_level();
    _is_uptimer_update_event = true;
    _is_rising_edge = false;
    _is_falling_edge = false;
    receive_handler();
}

inline void irreceiver_timer_ic_handler()
{
    if(__HAL_TIM_GET_FLAG(ptim_input_capture, TIM_FLAG_CC1) != RESET)
    {
        if(__HAL_TIM_GET_IT_SOURCE(ptim_input_capture, TIM_IT_CC1) != RESET)
        {
            debug_0_to_1_edge();

            // workaround for reset in slave mode not working? TODO
            ptim_input_capture->Instance->CNT=0;
            _ccr1 = ptim_input_capture->Instance->CCR1;

            if(_is_direct_logic)
            {
                _is_rising_edge = true;
                _is_falling_edge = false;
            }
            else
            {
                _is_rising_edge = false;
                _is_falling_edge = true;
            }
        }

    }
    else if(__HAL_TIM_GET_FLAG(ptim_input_capture, TIM_FLAG_CC2) != RESET)
    {
        if(__HAL_TIM_GET_IT_SOURCE(ptim_input_capture, TIM_IT_CC2) != RESET)
        {
            debug_1_to_0_edge();

            ptim_input_capture->Instance->CNT=0;
            _ccr2 = ptim_input_capture->Instance->CCR2;

            if(_is_direct_logic)
            {
                _is_falling_edge = true;
                _is_rising_edge = false;
            }
            else
            {
                _is_rising_edge = true;
                _is_falling_edge = false;
            }
        }
    }

    _is_uptimer_update_event = false;
    LineLevelState = LINE_UNDEFINED;

    receive_handler();
    // reset rising/falling edge vars
    _is_rising_edge = false;
    _is_falling_edge = false;

}

void setup_ic_timer()
{

    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_SlaveConfigTypeDef sSlaveConfig;
    TIM_IC_InitTypeDef sConfigIC;

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = envelop_timer_prescaler;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = max_period;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_IC_Init(&htim4);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);

    /// TIM_TI1_SetConfig
    //  ? Select the active input for TIMx_CCR1: write the CC1S bits to 01 in the TIMx_CCMR1 register (TI1 selected).
    //  ? Select the active polarity for TI1FP1 (used both for capture in TIMx_CCR1 and counter clear):
    //    write the CC1P bit to ‘0’ (active on rising edge).
    sConfigIC.ICFilter = 0;
    //TODO: cleanip?
    if(_is_direct_logic)
    {
        sConfigIC.ICPolarity = TIM_ICPOLARITY_RISING;
    }
    else
    {
        sConfigIC.ICPolarity = TIM_ICPOLARITY_FALLING;
    }

    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1);

    //  ? Select the active input for TIMx_CCR2: write the CC2S bits to 10 in the TIMx_CCMR1  register (TI1 selected).
    //  ? Select the active polarity for TI1FP2 (used for capture in TIMx_CCR2): write the CC2P bit to ‘1’ (active on falling edge).
    sConfigIC.ICFilter = 0;
    //TODO: cleanip?
    if(_is_direct_logic)
    {
        sConfigIC.ICPolarity = TIM_ICPOLARITY_FALLING;
    }
    else
    {
        sConfigIC.ICPolarity = TIM_ICPOLARITY_RISING;
    }

    sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
    HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2);//TIM_CHANNEL_2? TODO
    //  ? Select the valid trigger input: write the TS bits to 101 in the TIMx_SMCR register (TI1FP1 selected).
    //  ? Configure the slave mode controller in reset mode: write the SMS bits to 100 in the TIMx_SMCR register.

    sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
    sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
    //sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
    //TODO: cleanip?
    if(_is_direct_logic)
    {
        sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
    }
    else
    {
        sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_FALLING;
    }


    //TODO: why configuring reset breaks the thing?
    // why it does work without reset?
    //HAL_TIM_SlaveConfigSynchronization(&htim4, &sSlaveConfig);
    //HAL_TIM_SlaveConfigSynchronization_IT(&htim4, &sSlaveConfig); // TODO

    //sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);


    //  ? Enable the captures: write the CC1E and CC2E bits to ‘1’ in the TIMx_CCER register.

}

static inline void receive_handler()
{
    /* Data packet format: [preamble] [data frame] [epilogue]
     * preamble format: [longbit] [delay1] [bit1] [delay2]
     * data frame format: [word1] [delay] [word2] [delay] [word3]
     * data frame format: [delay1] [bit1] [delay2] [longbit]
     *
     *                      |<--     Preamble          -->|<--              data frame              -->|<--        epilogue  -->|
     *                       ______________      ____      ________          ________          ________      ____      _____________
     *                      |              |    |    |    |        |delay   |        |        |        |    |    |    |             |
     *                      |              |    |    |    |dataword|between |        |        |        |    |    |    |             |
     *                      |     750      |350 |350 |350 |  500   |words   |        |        |  500   |350 |350 |350 |    750      |
     *  ____________________|              |____|    |____|        |__500___|        |__....__|        |____|    |____|             |____
     *
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
    switch(ReceiverState)
    {
        case RX_WAITING_FOR_START_BIT:
        {
            // ensure than last data frame was DelayBetweenDataFrames ticks before
            if(is_0_to_1_edge() ) // rising edge -> bit may have started
            {
                if(is_correct_timming_interframe_delay()) //TODO: check delay before data frame
                {
                    debug_interframe_delay();

                    ptim_data_read->Instance->ARR = PreambleTotalLength; // will update on the "rising edge" of first data bit
                    ptim_data_read->Instance->CNT = 0;
                    ReceiverState = RX_START_BIT_PROCESSING;
                    StartStopSequenceReceiveState = Rx_PREAMBLE_BIT_1;
                    break;
                }
            }
            // no necessity in reset_receiver_state here, because nothing to change
            break;
        }
        case RX_START_BIT_PROCESSING:
        {
            // Start sequence consists of signal sequence {1,0,1,0}, each bit of length PeriodOfStartStopBits
            switch(StartStopSequenceReceiveState)
            {
                case Rx_PREAMBLE_BIT_1:
                {
                    if(is_1_to_0_edge()) // falling edge -> bit finished
                    {
                        if(is_correct_timming_preamble_bit()) // verify that it is within time interval boundaries
                        {
                            StartStopSequenceReceiveState = Rx_PREAMBLE_DELAY_1;
                            break;
                        }
                    }
                    // 1) probing timer is not used here 2) should not be dataread timer event (it is either stopped or waiting for when data should begin)
                    // Thus, need reset if event is different than what has been processed above
                    reset_receiver_state();
                    break;
                }
                case Rx_PREAMBLE_DELAY_1:
                {
                    if(is_0_to_1_edge()) // rising edge -> delay finished
                    {
                        if(is_correct_timming_preamble_delay()) // verify that it is within time interval boundaries
                        {
                            StartStopSequenceReceiveState = Rx_PREAMBLE_BIT_2;
                            break;
                        }
                    }
                    // 1) probing timer is not used here 2) should not be dataread timer event (it is either stopped or waiting for when data should begin)
                    // Thus, need reset if event is different than what has been processed above
                    reset_receiver_state();
                    break;
                }
                case Rx_PREAMBLE_BIT_2:
                {
                    if(is_1_to_0_edge()) // falling edge -> bit finished
                    {
                        if(is_correct_timming_preamble_bit()) // verify that it is within time interval boundaries
                        {
                            // Here is last preamble edge
                            // After second preamble bit there is last preamble delay, after which data will begin
                            // Thus, need to start dataread timer here (using last edge as sync event)
                            //HAL_TIM_Base_Start_IT(ptim_data_read);
                            ptim_data_read->Instance->CNT = 0;
                            ptim_data_read->Instance->ARR = PreambleDelayLength;
                            StartStopSequenceReceiveState = Rx_PREAMBLE_DELAY_2;
                            break;
                        }
                    }
                    reset_receiver_state();
                    break;
                }
                case Rx_PREAMBLE_DELAY_2:
                {
                    //FIXME TODO ISSUE

                    // this time it should be update event for dataread timer started on previous step
                    if(_is_uptimer_update_event) // delay finished
                    {
                        /// don't check delay length here

                        // TODO FIXME: why the hell it needs that much time of delay???
                        // At this point we are expected to be at the very beginning of the data frame, so
                        // to start reading data bits need to wait exactly HalfDataBitLength ticks
                        // But we are out of sync with offset of DataBitLength, for some reason
                        ptim_data_read->Instance->ARR = HalfDataBitLength;//DataBitLength + DataBitLength;
                        // preamble is received, stopping timer verifying preamble bits

                        StartStopSequenceReceiveState = Rx_PREAMBLE_BIT_1; // TODO: needless?
                        ReceiverState = RX_DATA_PROCESSNG;
                        DataFrameState = DATAFRAME_1_BEAMER_ID;
                        // initialize buffer with all zeros
                        // TODO: fill buffer with zeros
                        //memset(&data_frame, 0, sizeof(DataFrame_t));
                        rx_data_frame._1_beamer_id = 0;
                        rx_data_frame._2_angle_code=0;
                        rx_data_frame._3_angle_code_rev = 0;
                        // reset positions
                        rx_current_bit_pos = 0;

                        debug_preamble_end();

                        break;
                    }
                    // should not reset if not update event, since on the edge of preamble delay and data bit (may be rising edge)
                    break;
                }
                default:
                {
                    reset_receiver_state();
                    break;
                }
            }
            break;
        }
        case RX_DATA_PROCESSNG:
        {
            if(_is_uptimer_update_event)
            {
                debug_reading_data();

                switch(DataFrameState)
                {
                    case DATAFRAME_1_BEAMER_ID:
                    {
                        if(0 == rx_current_bit_pos) // execute only once, when first bit is being processed
                        {
                            // change period only when the very first bit of the whole data frame is being processed
                            ptim_data_read->Instance->ARR = DataBitLength;
                            rx_total_bits = sizeof(rx_data_frame._1_beamer_id) * 8;
                        }
                        // send current bit of current byte
                        if(rx_current_bit_pos < rx_total_bits)  // change to next state
                        {
                            decode_bit( &(rx_data_frame._1_beamer_id) );
                            rx_current_bit_pos++;
                        }
                        // move to next state and wait a delay between data fields
                        else
                        {
                            DataFrameState = DATAFRAME_2_ANGLE;
                            rx_current_bit_pos = 0;
                        }
                        break;
                    }
                    case DATAFRAME_2_ANGLE:
                    {
                        if(0 == rx_current_bit_pos) // execute only once, when first bit is being processed
                        {
                            rx_total_bits = sizeof(rx_data_frame._2_angle_code) * 8;
                        }
                        // send current bit of current byte
                        if(rx_current_bit_pos < rx_total_bits)  // change to next state
                        {
                            decode_bit( &(rx_data_frame._2_angle_code) );
                            rx_current_bit_pos++;
                        }
                        else
                        {
                            // change state to process second part of the data frame
                            DataFrameState = DATAFRAME_3_ANGLE_REV;
                            // reset current bit position
                            rx_current_bit_pos = 0;
                        }
                        break;
                    }
                    case DATAFRAME_3_ANGLE_REV:
                    {
                        // calculate total bit number only once, when first bit is being processed
                        if(0 == rx_current_bit_pos)
                        {
                            rx_total_bits = sizeof(rx_data_frame._3_angle_code_rev) * 8;
                        }

                        // send current bit of current byte
                        if(rx_current_bit_pos < rx_total_bits)  // change to next state
                        {
                            decode_bit( &(rx_data_frame._3_angle_code_rev) );
                            rx_current_bit_pos++;
                        }

                        // when all bits are done, go to next state
                        if(rx_current_bit_pos == rx_total_bits)
                        {
                            debug_data_end();

                            rx_current_bit_pos = 0;
                            // waiting last HalfDataBitLength for beginning of epilogue
                            ptim_data_read->Instance->ARR = HalfDataBitLength;
                            ReceiverState = RX_STOP_BIT_PROCESSING;
                            StartStopSequenceReceiveState = Rx_PREAMBLE_START;
                        }
                        break;
                    }
                    default:
                    {
                        reset_receiver_state();
                        break;
                    }
                }
            }// end of _is_timer_update
            /// else - input capture during data receiving, nothing to do.
            /// TODO: turn off input capture timer when it is not supposed to be used
            break;
        }
        case RX_STOP_BIT_PROCESSING:
        {
            switch (StartStopSequenceReceiveState)
            {
                //low: off confirmation
                case Rx_PREAMBLE_START:
                {
                    // start verifying first delay
                    if(_is_uptimer_update_event)
                    {
                        debug_epilogue_begin();
                        ptim_input_capture->Instance->CNT=0;
                        ptim_data_read->Instance->CNT = 0;
                        ptim_data_read->Instance->ARR = max_period;
                        StartStopSequenceReceiveState = Rx_PREAMBLE_DELAY_1;
                        break;
                    }
                    // should not reset if not update event, since on the edge of data bit and preamble delay  (may be falling edge)
                    break;
                }
                case Rx_PREAMBLE_DELAY_1:
                {
                    if(is_0_to_1_edge()) // rising edge -> delay finished, bit started
                    {
                        if(is_correct_timming_preamble_delay()) // check delay length in allowed interval
                        {
                            StartStopSequenceReceiveState = Rx_PREAMBLE_BIT_1;
                            break;
                        }
                    }
                    // currently, we are in the edge of last data bit and preamble/epilogue delay.
                    // Since data bit may be 1, it may be falling edge near data bit end, instead of rising edge near preamble/epilogue bit start
                    // FIXME TODO: rearrange checks to avoid rising/falling collision
                    //reset_receiver_state();
                    break;
                }
                    // low falling edge: STAGE_ON2 -> STAGE_OFFF2
                case Rx_PREAMBLE_BIT_1:
                {
                    if(is_1_to_0_edge())// falling edge -> bit finished
                    {
                        if(is_correct_timming_preamble_bit())// check bit length in allowed interval
                        {
                            process_received_data();

                            debug_data_received();
                        }
                    }
                    reset_receiver_state();
                    break;
                }
                default:
                {
                    reset_receiver_state();
                    break;
                }
            }// end of switch (StartStopSequenceReceiveState)
            break;
        }
        default:
        {
            reset_receiver_state();
            break;
        }
    } // switch(ReceiverState)
}

static inline void reset_receiver_state()
{
    //HAL_TIM_IC_PWM_Start_IT(&htim4); //TODO
    ReceiverState = RX_WAITING_FOR_START_BIT;
    DataFrameState = DATAFRAME_1_BEAMER_ID;
    ptim_input_capture->Instance->ARR = max_period;
    ptim_input_capture->Instance->CNT = 0;
    ptim_data_read->Instance->ARR = max_period;
    ptim_data_read->Instance->CNT = 0;

    {
        //HAL_GPIO_WritePin(GPIO_LED_PORT,GPIO_LED_PIN,GPIO_PIN_RESET);
    }

    //HAL_TIM_Base_Stop_IT(ptim_data_read);
}

static inline void process_received_data()
{
    /// we successfully received data, send corresponding event for listeners to read from the data buffer
    /// verify correctness
    data_frame_delta = rx_data_frame._2_angle_code ^ (~rx_data_frame._3_angle_code_rev);
    if( data_frame_delta == 0 )
    {
        debug_data_verified();

        copy_data_frame_to_buffer(&rx_data_frame);
        // TODO: change to the real thing
        const char* test_str = "HelloWireless!\0";
        memcpy(tx_buf, test_str, strlen(test_str));

        //TODO: check and remove send_dataready_signal();

        {
            TXX();
            HAL_GPIO_TogglePin(GPIO_LED_PORT,GPIO_LED_PIN);
            //HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
        }
    }
    else
    {
        //TODO
    }
}

static inline void copy_data_frame_to_buffer(DataFrame_t* df)
{
    data_frames[arr_index]._2_angle_code = df->_2_angle_code;
    data_frames[arr_index]._1_beamer_id = df->_1_beamer_id;
    data_frames[arr_index]._3_angle_code_rev = df->_3_angle_code_rev;
    arr_index++;
    //memcpy(df, rx_data_frame_array, sizeof(rx_data_frame));
}

static inline bool is_1_to_0_edge()
{
    //return _is_falling_edge;
    //TODO:FIXME: review
    if(_is_direct_logic)
    {
        return _is_falling_edge;
    }
    return _is_rising_edge;
}

static inline bool is_0_to_1_edge()
{
    //return _is_rising_edge;
    //TODO:FIXME: review
    if(_is_direct_logic)
    {
        return _is_rising_edge;
    }
    return _is_falling_edge;
}

static inline bool is_1_on_update_event()
{
    if(_is_direct_logic)
    {
        return (LINE_HIGH_ON_UPDATE_EVENT == LineLevelState);
    }
    return (LINE_LOW_ON_UPDATE_EVENT == LineLevelState);
}

static inline bool is_0_on_update_event()
{
    if(_is_direct_logic)
    {
        return (LINE_LOW_ON_UPDATE_EVENT == LineLevelState);
    }
    return (LINE_HIGH_ON_UPDATE_EVENT == LineLevelState);
}

static inline void get_logical_level()
{
    if(HAL_GPIO_ReadPin(GPIO_PORT_IR_IN, GPIO_PIN_IR_IN) == GPIO_PIN_SET)
    {
        LineLevelState = LINE_HIGH_ON_UPDATE_EVENT;
    }
    else if(HAL_GPIO_ReadPin(GPIO_PORT_IR_IN, GPIO_PIN_IR_IN) == GPIO_PIN_RESET)
    {
        LineLevelState = LINE_LOW_ON_UPDATE_EVENT;
    }
    else
    {
        LineLevelState = LINE_UNDEFINED;
        return;
    }
}

static inline bool is_correct_timming_interframe_delay()
{
    // check only lower bound of delay.
    return (_ccr1 > InterframeDelayLength - max_delta_interframe_delay);
}

static inline bool is_correct_timming_preamble_bit()
{
    return (_ccr2 < PreambleBitLength + max_delta_preamble_bit)
            &&
            (_ccr2 > PreambleBitLength - max_delta_preamble_bit);
}

static inline bool is_correct_timming_preamble_delay()
{
    return (_ccr1 < PreambleDelayLength + max_delta_preamble_delay)
            &&
            (_ccr1 > PreambleDelayLength - max_delta_preamble_delay);
}

static inline void send_dataready_signal()
{
    /// TODO: Implement me
    /// Prerequisets:
    /// 1. Each sensor has ID
    /// 2. All sensors are aware of total number of sensors in the array
    /// 3. Hub
    /// Pseudocode for syncronization among sensors and hub

    // There is an array of sensors whith ID = [1..N]
    // All sensors initially are in listening mode

    // Each sensor checks its ID. If it is first in the array (ID == 1), it first sends message with its ID and data, without waiting
    // Hub and all remaining sensors are waiting for receiving the message
    // Each sensor checks if it is his turn to send data (its Id is the next ID after that which has been received).
    // If his ID is the next, it sends the latest data
    // if (ID_from_msg == ID_self - 1):
    //      send data()
    // When all sensors have sent data (last ID == N), process should be repeated.
    // So the first sensor (with ID == 1) checks if ID_msg == N, if so - it is next to send data


    //send_data_uart( (uint8_t *)&rx_data_frame, sizeof(rx_data_frame));
}

static inline void dbg_pulse_1()
{
#ifdef DEBUG
    //TODO: clean HAL_GPIO_TogglePin(DBG_OUT_1_Port, DBG_OUT_1_Pin);
#endif//DEBUG
}

static inline void dbg_pulse_2()
{
#ifdef DEBUG
    //TODO: clean HAL_GPIO_TogglePin(DBG_OUT_2_Port, DBG_OUT_2_Pin);
#endif
}

static inline void debug_interframe_delay() {
    if( DEBUG_FRAME_DELAY_1)
        dbg_pulse_1();
    if( DEBUG_FRAME_DELAY_2)
        dbg_pulse_2();
}

static inline void debug_reading_data() {
    if( DEBUG_READING_DATA_1)
        dbg_pulse_1();
    if( DEBUG_READING_DATA_2)
        dbg_pulse_2();
}

static inline void debug_data_verified() {
    if( DEBUG_DATA_VERIFIED_1)
        dbg_pulse_1();
    if( DEBUG_DATA_VERIFIED_2)
        dbg_pulse_2();
}

static inline void debug_data_end() {
    if( DEBUG_DATA_END_1)
        dbg_pulse_1();
    if( DEBUG_DATA_END_2)
        dbg_pulse_2();
}

static inline void debug_data_received() {
    if( DEBUG_DATA_RECEIVED_1)
        dbg_pulse_1();
    if( DEBUG_DATA_RECEIVED_2)
        dbg_pulse_2();
}

static inline void debug_upd_event() {
    if( DEBUG_UPD_EVENT_1)
        dbg_pulse_1();
    if( DEBUG_UPD_EVENT_2)
        dbg_pulse_2();
}

static inline void debug_0_to_1_edge() {
    if( DEBUG_0_to_1_EDGE_1)
        dbg_pulse_1();
    if( DEBUG_0_to_1_EDGE_2)
        dbg_pulse_2();
}

static inline void debug_1_to_0_edge() {
    if( DEBUG_1_to_0_EDGE_1)
        dbg_pulse_1();
    if( DEBUG_1_to_0_EDGE_2)
        dbg_pulse_2();
}

static inline void debug_epilogue_begin() {
    if( DEBUG_EPILOGUE_BEGIN_1)
        dbg_pulse_1();
    if( DEBUG_EPILOGUE_BEGIN_2)
        dbg_pulse_2();
}

static inline void debug_epilogue_end() {
    if( DEBUG_EPILOGUE_END_1)
        dbg_pulse_1();
    if( DEBUG_EPILOGUE_END_2)
        dbg_pulse_2();
}

static inline void debug_preamble_end() {
    if( DEBUG_PREAMBLE_END_1)
        dbg_pulse_1();
    if( DEBUG_PREAMBLE_END_2)
        dbg_pulse_2();
}
