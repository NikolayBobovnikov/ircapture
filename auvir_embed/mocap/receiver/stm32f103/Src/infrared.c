#include "infrared.h"

/// Parameters
extern TIM_HandleTypeDef* ptim_input_capture;
extern TIM_HandleTypeDef* ptim_data_read;
extern GPIO_TypeDef * GPIO_PORT_IR_IN;
extern uint16_t GPIO_PIN_IR_IN;
extern const bool _is_direct_logic;

// TODO: parametrize values below

///====================== Variables ======================
// main buffer for storing recent data frames
#define  RX_BUF_SIZE 10
DataFrame_t data_frames[RX_BUF_SIZE] = {0}; // TODO: verify initialization
uint8_t arr_index = 0;

DataFrame_t rx_data_frame;
volatile uint8_t ReceiverState = RX_WAITING_FOR_START_BIT;
volatile uint8_t StartStopSequenceReceiveState = STAGE_PREAMBLE_START;
volatile uint8_t DataFrameState = DATAFRAME_1_BEAMER_ID;
volatile uint8_t LineLevelState = LINE_UNDEFINED;

uint8_t rx_data = 0;
volatile size_t rx_total_bits = 0;
volatile uint8_t rx_current_bit_pos = 0;
volatile uint8_t rx_bit = 0;

volatile bool _is_rising_edge = false;
volatile bool _is_falling_edge = false;
volatile bool _is_uptimer_update_event = false;

volatile uint16_t _ccr1 = 0;
volatile uint16_t _ccr2 = 0;
volatile uint16_t _delay_counter = 0;


// TODO: cleanup after debugging
int dbg[1000]={0};
int dbg_index=0;

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
//#define DEBUG_0_to_1_EDGE_1
//#define DEBUG_0_to_1_EDGE_2
//#define DEBUG_1_to_0_EDGE_1
//#define DEBUG_1_to_0_EDGE_2
//#define DEBUG_CHECK_IC_TIMING_1
//#define DEBUG_CHECK_IC_TIMING_2

//#define DEBUG_EDGES
//#define DEBUG_DATA

#ifdef DEBUG_EDGES
    #define DEBUG_0_to_1_EDGE_2
    #define DEBUG_1_to_0_EDGE_1
#endif

#ifdef DEBUG_DATA
#define DEBUG_READING_DATA_2
#define DEBUG_DATA_RECEIVED_1
#endif



///====================== Private function declarations ======================
// main routine called from timer interrupts to manage receiving process
static inline void receive_handler();

// helper functions
static inline bool is_1_to_0_edge();
static inline bool is_0_to_1_edge();
static inline bool is_1_on_update_event();
static inline bool is_0_on_update_event();
static inline void reset_receiver_state();
static inline void send_dataready_signal();
static inline void get_logical_level();

// function to copy data frame to the main buffer, when data is received successfully
static inline void copy_data_frame_to_buffer(DataFrame_t* df);

static inline bool is_correct_timming_interframe_delay();
static inline bool is_correct_timming_preamble_bit();
static inline bool is_correct_timming_preamble_delay();


// for debugging. TODO: cleanup when done
static inline void dbg_pulse_1();
static inline void dbg_pulse_2();

///====================== Functions ======================

inline void irreceiver_timer_up_handler()
{
    get_logical_level();
    _is_uptimer_update_event = true;
    _is_rising_edge = false;
    _is_falling_edge = false;
    receive_handler();

#ifdef DEBUG_UPD_EVENT_1
    dbg_pulse_1();
#endif
#ifdef DEBUG_UPD_EVENT_2
    dbg_pulse_2();
#endif

}
inline void irreceiver_timer_ic_handler()
{
    if(__HAL_TIM_GET_FLAG(ptim_input_capture, TIM_FLAG_CC1) != RESET)
    {
        if(__HAL_TIM_GET_IT_SOURCE(ptim_input_capture, TIM_IT_CC1) != RESET)
        {
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

#ifdef DEBUG_0_to_1_EDGE_1
            dbg_pulse_1();
#endif
#ifdef DEBUG_0_to_1_EDGE_2
            dbg_pulse_2();
#endif
        }

    }
    else if(__HAL_TIM_GET_FLAG(ptim_input_capture, TIM_FLAG_CC2) != RESET)
    {
        if(__HAL_TIM_GET_IT_SOURCE(ptim_input_capture, TIM_IT_CC2) != RESET)
        {
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

#ifdef DEBUG_1_to_0_EDGE_2
            dbg_pulse_2();
#endif
#ifdef DEBUG_1_to_0_EDGE_1
            dbg_pulse_1();
#endif
        }
    }

    _is_uptimer_update_event = false;
    LineLevelState = LINE_UNDEFINED;

    receive_handler();
    // reset rising/falling edge vars
    _is_rising_edge = false;
    _is_falling_edge = false;

}

static inline void receive_handler()
{
    if(dbg_index >= 998)
    {
        dbg_index = 0;
    }
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
                    ReceiverState = RX_START_BIT_PROCESSING;
                    StartStopSequenceReceiveState = STAGE_PREAMBLE_BIT_1;
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
                case STAGE_PREAMBLE_BIT_1:
                {
                    if(is_1_to_0_edge()) // falling edge -> bit finished
                    {
                        if(is_correct_timming_preamble_bit()) // verify that it is within time interval boundaries
                        {
                            StartStopSequenceReceiveState = STAGE_PREAMBLE_DELAY_1;
                            break;
                        }
                    }
                    // 1) probing timer is not used here 2) should not be dataread timer event (it is either stopped or waiting for when data should begin)
                    // Thus, need reset if event is different than what has been processed above
                    reset_receiver_state();
                    break;
                }
                case STAGE_PREAMBLE_DELAY_1:
                {
                    if(is_0_to_1_edge()) // rising edge -> delay finished
                    {
                        if(is_correct_timming_preamble_delay()) // verify that it is within time interval boundaries
                        {
                            StartStopSequenceReceiveState = STAGE_PREAMBLE_BIT_2;
                            break;
                        }
                    }
                    // 1) probing timer is not used here 2) should not be dataread timer event (it is either stopped or waiting for when data should begin)
                    // Thus, need reset if event is different than what has been processed above
                    reset_receiver_state();
                    break;
                }
                case STAGE_PREAMBLE_BIT_2:
                {
                    if(is_1_to_0_edge()) // falling edge -> bit finished
                    {
                        if(is_correct_timming_preamble_bit()) // verify that it is within time interval boundaries
                        {
                            // Here is last preamble edge
                            // After second preamble bit there is last preamble delay, after which data will begin
                            // Thus, need to start dataread timer here (using last edge as sync event)
                            HAL_TIM_Base_Start_IT(ptim_data_read);
                            ptim_data_read->Instance->CNT = 0;
                            ptim_data_read->Instance->ARR = PreambleDelayLength;
                            StartStopSequenceReceiveState = STAGE_PREAMBLE_DELAY_2;
                            break;
                        }
                    }
                    reset_receiver_state();
                    break;
                }
                case STAGE_PREAMBLE_DELAY_2:
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
                        ptim_data_read->Instance->ARR = DataBitLength + DataBitLength;
                        // preamble is received, stopping timer verifying preamble bits

                        StartStopSequenceReceiveState = STAGE_PREAMBLE_BIT_1; // TODO: needless?
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
                        break;
                    }
                    reset_receiver_state();
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
                            // k-th bit of n: (n >> k) & 1
                            if(is_1_on_update_event())
                            {
                                // set bit at the inversed position
                                rx_data_frame._1_beamer_id |= 1
                                        << (rx_total_bits - rx_current_bit_pos - 1);
                                        //<< (rx_current_bit_pos);
                            }
                            /// no need to set bit to zero if signal is low, since all bits are initialized to zeros
                            rx_current_bit_pos++;
                        }
                        // move to next state and wait a delay between data fields
                        else
                        {
                            // change state to process second part of the data frame
                            DataFrameState = DATAFRAME_2_ANGLE;
                            // reset current bit position
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
                            // k-th bit of n: (n >> k) & 1
                            if(is_1_on_update_event())
                            {
                                // set bit at the inversed position
                                rx_data_frame._2_angle_code |= 1
                                        << (rx_total_bits - rx_current_bit_pos - 1);
                                        //<< (rx_current_bit_pos);
                            }
                            /// no need to set bit to zero if signal is low, since all bits are initialized to zeros
                            rx_current_bit_pos++;
                        }
                        else
                        {
                            // change state to process second part of the data frame
                            DataFrameState = DATAFRAME_3_TIME;
                            // reset current bit position
                            rx_current_bit_pos = 0;
                        }
                        break;
                    }
                    case DATAFRAME_3_TIME:
                    {
                        if(0 == rx_current_bit_pos) // execute only once, when first bit is being processed
                        {
                            rx_total_bits = sizeof(rx_data_frame._3_angle_code_rev) * 8;
                        }
                        // send current bit of current byte
                        if(rx_current_bit_pos < rx_total_bits)  // change to next state
                        {
                            // k-th bit of n: (n >> k) & 1
                            if(is_1_on_update_event())
                            {
                                // set bit at the inversed position
                                rx_data_frame._3_angle_code_rev |= 1
                                        << (rx_total_bits - rx_current_bit_pos - 1);
                                        //<< (rx_current_bit_pos);
                            }
                            /// no need to set bit to zero if signal is low, since all bits are initialized to zeros
                            rx_current_bit_pos++;
                        }

                        if(rx_current_bit_pos == rx_total_bits)
                        {
                            rx_current_bit_pos = 0;
                            // waiting for beginning of epilogue
                            ptim_data_read->Instance->ARR = HalfDataBitLength;
                            ReceiverState = RX_STOP_BIT_PROCESSING;
                            StartStopSequenceReceiveState = STAGE_PREAMBLE_START;
                        }
                        break;
                    }

                }

#ifdef DEBUG_READING_DATA_1
                            dbg_pulse_1();
#endif
#ifdef DEBUG_READING_DATA_2
                            dbg_pulse_2();
#endif
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
                case STAGE_PREAMBLE_START:
                {
                    // start verifying first short delay
                    if(_is_uptimer_update_event)
                    {
                        ptim_input_capture->Instance->CNT=0;
                    	HAL_TIM_Base_Stop_IT(ptim_data_read);
                        StartStopSequenceReceiveState = STAGE_PREAMBLE_DELAY_1;
                        break;
                    }
                    /// Don't need to reset here
                    break;
                }
                //high rising edge: STAGE_OFF1 -> STAGE_ON1
                case STAGE_PREAMBLE_DELAY_1:
                {
                    if(is_0_to_1_edge()) // rising edge -> delay finished, bit started
                    {
                        if(is_correct_timming_preamble_delay()) // check delay length in allowed interval
                        {
                            StartStopSequenceReceiveState = STAGE_PREAMBLE_BIT_1;
                            break;
                        }
                    }
                    reset_receiver_state();
                    break;
                }
                // low falling edge: STAGE_ON2 -> STAGE_OFFF2
                case STAGE_PREAMBLE_BIT_1:
                {
                    if(is_1_to_0_edge())// falling edge -> bit finished
                    {
                        if(is_correct_timming_preamble_bit())// check bit length in allowed interval
                        {
                            StartStopSequenceReceiveState = STAGE_PREAMBLE_DELAY_2;
                            break;
                        }
                    }
                    reset_receiver_state();
                    break;
                }
                // high rising edge: STAGE_OFF1 -> STAGE_ON2
                case STAGE_PREAMBLE_DELAY_2:
                {
                    // rising edge input capture
                    if(is_0_to_1_edge())// rising edge -> delay finished, bit started
                    {
                        if(is_correct_timming_preamble_delay())// check delay length in allowed interval
                        {
                             StartStopSequenceReceiveState = STAGE_PREAMBLE_BIT_2;
                             break;
                        }
                    }
                    reset_receiver_state(); //TODO: check if it needed
                    break;
                }
                //low falling edge: STAGE_ON1 -> STAGE_OFF1
                case STAGE_PREAMBLE_BIT_2:
                {
                    if(is_1_to_0_edge())// falling edge -> bit finished
                    {
                        if(is_correct_timming_preamble_bit())// check bit length in allowed interval
                        {
                            ReceiverState = RX_STOP_BIT_DONE;
                            StartStopSequenceReceiveState = STAGE_PREAMBLE_START;
                            break;
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
            }
            break;
        }
        case RX_STOP_BIT_DONE:
        {
            // immediately after stop sequence, line should be low
            // low confirmation
            //if(is_0_on_update_event()) TODO: need to start dataread timer for this.
            {
                // we successfully received data, send corresponding event for listeners to read from the data buffer

                /// verify correctness
                if( 0 == (rx_data_frame._2_angle_code ^ ~rx_data_frame._3_angle_code_rev))
                {
                    copy_data_frame_to_buffer(&rx_data_frame);
                    send_dataready_signal();
                    dbg[dbg_index] = rx_data_frame._2_angle_code;
                    dbg[dbg_index+1] = rx_data_frame._3_angle_code_rev;
                    dbg_index = dbg_index + 2;
                }
                else
                {
                    //TODO
                }



#ifdef DEBUG_DATA_RECEIVED_1
                dbg_pulse_1();
#endif
#ifdef DEBUG_DATA_RECEIVED_2
                dbg_pulse_2();
#endif
            }
            reset_receiver_state();
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
    ptim_input_capture->Instance->ARR = 65535;
    ptim_input_capture->Instance->CNT = 0;
    HAL_TIM_Base_Stop_IT(ptim_data_read);
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
    return (_ccr1 < InterframeDelayLength + max_delta_interframe_delay)
            &&
           (_ccr1 > InterframeDelayLength - max_delta_interframe_delay);
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

    // Each sensor checks its ID. If it is first in the array (ID == 1), it first sends message with its ID and data.
    // Hub and all remaining sensors receives the message
    // Each sensor checks if it is his turn to send data (its Id is the next ID after that which has been received).
    // If his ID is the next, it sends the latest data
    // if (ID_from_msg == ID_self - 1):
    //      send data()
    // When all sensors have sent data (last ID == N), process should be repeated.
    // So the first sensor (with ID == 1) checks if ID_msg == N, if so - it is next to send data
}
static inline void dbg_pulse_1()
{
#ifdef DEBUG
    //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_SET);
    //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_RESET);
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
#endif//DEBUG
}
static inline void dbg_pulse_2()
{
#ifdef DEBUG
    //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_SET);
    //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_RESET);
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
#endif
}

