#include "infrared.h"
// b infrared.c:

/// Parameters
extern TIM_HandleTypeDef* ic_tim_p;
extern TIM_HandleTypeDef* up_tim_p;
extern GPIO_TypeDef * GPIO_PORT_IR_IN;
extern uint16_t GPIO_PIN_IR_IN;
extern const bool _is_direct_logic;

/*
 *  Constants
*/
// FIXME: TODO: keep values below in sync with transmitter
const uint16_t envelop_timer_prescaler = 72 - 1;    // values below are for prescaler=14
const uint16_t StartStopBitLength = 500 - 1;    // 700 works not reliably; 750 works; 800 chosen
const uint16_t DataBitLength = 1000 - 1;        // TODO: justify value. Need to be distinguishable from start/stop bits
const uint16_t DelayBetweenDataFramesTotal = 20000 - 1;//56000 doesn't work; 56300, 56500, 57000 works; 58000 chosen




const uint16_t HalfDataBitLength = 500 - 1;
const uint16_t HalfStartStopBitLength = 250 - 1;
const uint16_t HalfStartStopHalfDataBitLength = 750 - 1;
const uint16_t StartStopBitPeriod = 1000 - 1;
const uint16_t DelayCheckingPeriod = 10 - 1;
const uint16_t max_delta_pwm = 200;
const uint16_t max_delta_pwm_width = 200;
const uint16_t max_delta_delay = 500; // TODO: cleanup?
//const uint8_t max_delta_cnt_delay = 10; TODO: cleanup or use
// TODO: parametrize values below
const uint16_t DelayBetweenDataFramesToCheck = 19750; // DelayBetweenDataFramesTotal - HalfStartStopBitLength;
const uint16_t DelayCounterMin = 1975 - 500; // (DelayBetweenDataFramesToCheck / actual DelayCheckingPeriod) - max_delta_cnt_delay;


/*
 *  Variables
*/

// main buffer for storing recent data frames
#define  RX_BUF_SIZE 10
DataFrame_t data_frames[RX_BUF_SIZE] = {0}; // TODO: verify initialization
uint8_t arr_index = 0;

DataFrame_t rx_data_frame;
volatile uint8_t ReceiverState = RX_WAITING_FOR_START_BIT;
volatile uint8_t StartStopSequenceReceiveState = STAGE_ON1;
volatile uint8_t DataFrameState = DATAFRAME_1_BEAMER_ID;
volatile uint8_t LineLevelState = LINE_UNDEFINED;

uint8_t rx_data = 0;
volatile size_t rx_total_bits = 0;
volatile uint8_t rx_current_bit_pos = 0;
volatile uint8_t rx_bit = 0;

volatile bool _is_rising_edge = false;
volatile bool _is_falling_edge = false;
volatile bool _is_timer_update_event = false;
volatile bool _is_interframe_delay_long_enough = false;
volatile uint16_t _ccr1 = 0;
volatile uint16_t _ccr2 = 0;
volatile uint16_t _delay_counter = 0;


// TODO: cleanup after debugging
int dbg[100]={0};
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

/*
 *  Function definitions
*/

inline void irreceiver_timer_up_handler()
{
    if(HAL_GPIO_ReadPin(GPIO_PORT_IR_IN, GPIO_PIN_IR_IN) == GPIO_PIN_SET)
    {
        LineLevelState = LINE_HIGH_ON_UPDATE_EVENT;
    }
    else if(HAL_GPIO_ReadPin(GPIO_PORT_IR_IN, GPIO_PIN_IR_IN) == GPIO_PIN_RESET)
    {
        LineLevelState = LINE_LOW_ON_UPDATE_EVENT;

#ifdef DEBUG_LOW_CHECK_2
        dbg_pulse_2();
#endif
#ifdef DEBUG_LOW_CHECK_1
        dbg_pulse_1();
#endif
    }
    else
    {
        LineLevelState = LINE_UNDEFINED;
        reset_delay_cnt();
        return;
    }
    _is_timer_update_event = true;
    _is_rising_edge = false;
    _is_falling_edge = false;
    update_delay_cnt();
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
    if(__HAL_TIM_GET_FLAG(ic_tim_p, TIM_FLAG_CC1) != RESET)
    {
        if(__HAL_TIM_GET_IT_SOURCE(ic_tim_p, TIM_IT_CC1) != RESET)
        {
            // workaround for reset in slave mode not working? TODO
            ic_tim_p->Instance->CNT=0;
            _ccr1 = ic_tim_p->Instance->CCR1;
            _is_rising_edge = true;
            _is_falling_edge = false;

#ifdef DEBUG_0_to_1_EDGE_1
            dbg_pulse_1();
#endif
#ifdef DEBUG_0_to_1_EDGE_2
            dbg_pulse_2();
#endif
        }

    }
    else if(__HAL_TIM_GET_FLAG(ic_tim_p, TIM_FLAG_CC2) != RESET)
    {
        if(__HAL_TIM_GET_IT_SOURCE(ic_tim_p, TIM_IT_CC2) != RESET)
        {
            _ccr2 = ic_tim_p->Instance->CCR2;
            _is_falling_edge = true;
            _is_rising_edge = false;

#ifdef DEBUG_1_to_0_EDGE_2
            dbg_pulse_2();
#endif
#ifdef DEBUG_1_to_0_EDGE_1
            dbg_pulse_1();
#endif
        }
    }
    _is_timer_update_event = false;
    LineLevelState = LINE_UNDEFINED;

    receive_handler();
    // reset helper vars
    _is_rising_edge = false;
    _is_falling_edge = false;
}

static inline void receive_handler()
{
    if(dbg_index == 100)
    {
        dbg_index = 0;
    }
    /* Receive data frame
     * 1. start sequence
     * 2. data
     *  2.1 Coded angle
     *  2.2 Time of emission
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

            if(is_0_to_1_edge() )
            {
                if(is_ic_after_interframe_delay()) //TODO: check delay before data frame
                {
                    // wait for first point
                    //HAL_TIM_Base_Start_IT(up_tim_p);
                    up_tim_p->Instance->CNT = 0;

                    up_tim_p->Instance->ARR = HalfStartStopBitLength;
                    ReceiverState = RX_START_BIT_PROCESSING;
                    StartStopSequenceReceiveState = STAGE_ON1;
                    // wait for half a period of startstop bit sequence
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
                //High confirmed: STAGE_0 -> STAGE_ON1 confirmation
                case STAGE_ON1:
                {
                    // This should be on update event
                    if(is_1_on_update_event())
                    {
                        // first point. change timer period to the period between reading start/stop bit values
                        up_tim_p->Instance->ARR = StartStopBitLength;
                        StartStopSequenceReceiveState = STAGE_ON1_OFF1;
                        break;
                    }
                    reset_receiver_state();
                    break;
                }
                //Low: STAGE_ON1 -> STAGE_OFF1 input capture
                case STAGE_ON1_OFF1:
                {
                    // falling edge should be detected
                    if(is_1_to_0_edge())
                    {
                        if(is_1_to_0_edge_timing_ok())
                        {
                            StartStopSequenceReceiveState = STAGE_OFF1;
                            break;
                        }
                    }
                    reset_receiver_state();
                    break;
                }
                //Low: STAGE_OFF1 confirmation
                case STAGE_OFF1:
                {
                    // This should be on update
                    if(is_0_on_update_event())
                    {
                        StartStopSequenceReceiveState = STAGE_OFF1_ON2;
                        // wait for half a period of startstop bit sequence
                        break;
                    }
                    reset_receiver_state();
                    break;
                }
                //High: STAGE_OFF1 -> STAGE_ON2 input capture
                case STAGE_OFF1_ON2:
                {
                    // This should be on IC event (2nd bit - rising edge)
                    if(is_0_to_1_edge())
                    {
                        if(is_0_to_1_edge_timing_ok())
                        {
                            StartStopSequenceReceiveState = STAGE_ON2;
                            // wait for half a period of startstop bit sequence
                            break;
                        }
                    }
                    reset_receiver_state();
                    break;
                }
                //High: STAGE_ON2 confirmation
                case STAGE_ON2:
                {
                    // This should be on update
                    if(is_1_on_update_event())
                    {
                        StartStopSequenceReceiveState = STAGE_ON2_OFF2;
                        // wait for half a period of startstop bit sequence
                        break;
                    }
                    reset_receiver_state();
                    break;
                }
                //Low: STAGE_ON2 -> STAGE_OFF2 input capture
                case STAGE_ON2_OFF2:
                {
                    // This should be on IC event (2nd bit - rising edge)
                    if(is_1_to_0_edge())
                    {
                        if(is_1_to_0_edge_timing_ok())
                        {
                            StartStopSequenceReceiveState = STAGE_OFF2;
                            break;
                        }
                    }
                    reset_receiver_state();
                    break;
                }
                //Low: STAGE_OFF2 confirmation
                case STAGE_OFF2:
                {
                    if(is_0_on_update_event())
                    {
                        // turn off input capture temporarily,
                        //wait for the beginning of data transmission
                        //HAL_TIM_IC_PWM_Stop_IT(&htim4); //TODO
                        StartStopSequenceReceiveState = STAGE_OFF2_ON3;
                        break;
                    }
                    reset_receiver_state();
                    break;
                }
                // High
                case STAGE_OFF2_ON3:
                {
                    // This should be on IC event (2nd bit - rising edge)
                    if(is_0_to_1_edge())
                    {
                        if(is_0_to_1_edge_timing_ok())
                        {
                            StartStopSequenceReceiveState = STAGE_ON3;
                            // wait for half a period of startstop bit sequence
                            break;
                        }
                    }
                    reset_receiver_state();
                    break;
                }
                //High: STAGE_ON3 confirmation
                case STAGE_ON3:
                {
                    // This should be on update
                    if(is_1_on_update_event())
                    {
                        StartStopSequenceReceiveState = STAGE_ON3_OFF4;
                        // wait for half a period of startstop bit sequence
                        break;
                    }
                    reset_receiver_state();
                    break;
                }
                //Low: STAGE_ON2 -> STAGE_OFF2 input capture
                case STAGE_ON3_OFF4:
                {
                    // This should be on IC event (2nd bit - rising edge)
                    if(is_1_to_0_edge())
                    {
                        if(is_1_to_0_edge_timing_ok())
                        {
                            StartStopSequenceReceiveState = STAGE_OFF4;
                            // restart a counter to reduce integrating of error,
                            // since we have a precise zero point of falling edge
                            // wait for half a period of startstop bit sequence
                            up_tim_p->Instance->CNT = 0;
                            up_tim_p->Instance->ARR = HalfStartStopBitLength;
                            break;
                        }
                    }
                    reset_receiver_state();
                    break;
                }
                //Low: STAGE_OFF2 confirmation
                case STAGE_OFF4:
                {
                    if(is_0_on_update_event())
                    {
                        // turn off input capture temporarily,
                        //wait for the beginning of data transmission
                        //HAL_TIM_IC_PWM_Stop_IT(&htim4); //TODO
                        StartStopSequenceReceiveState = STAGE_0;
                        ///ReceiverState = RX_START_BIT_DONE;

                        {
                            ///Prepare to read data frame
                            up_tim_p->Instance->ARR = HalfStartStopHalfDataBitLength;
                            // start reading data bits after the middle of the first pulse,
                            // so wait for another HalfPeriodOfDataBits
                            ReceiverState = RX_DATA_PROCESSNG;
                            DataFrameState = DATAFRAME_1_BEAMER_ID;
                            //ReceiverState = RX_DATA_PROCESSNG;
                            //DataFrameState= DATAFRAME_1_BEAMER_ID;
                            // initialize buffer with all zeros
                            // TODO: fill buffer with zeros
                            //memset(&data_frame, 0, sizeof(DataFrame_t));
                            rx_data_frame._1_beamer_id = 0;
                            rx_data_frame._2_angle_graycode=0;
                            rx_data_frame._3_timer_cnt = 0;
                            // reset positions
                            rx_current_bit_pos = 0;
                        }

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
        // FIXME: review the state below for usefullness ->
        case RX_START_BIT_DONE:
        {
            if(_is_timer_update_event)
            {
                ///Prepare to read data frame

                // start reading data bits after the middle of the first pulse,
                // so wait for another HalfPeriodOfDataBits
                up_tim_p->Instance->ARR = HalfDataBitLength;
                ReceiverState = RX_DATA_PROCESSNG;
                DataFrameState = DATAFRAME_1_BEAMER_ID;
                //ReceiverState = RX_DATA_PROCESSNG;
                //DataFrameState= DATAFRAME_1_BEAMER_ID;
                // initialize buffer with all zeros
                // TODO: fill buffer with zeros
                //memset(&data_frame, 0, sizeof(DataFrame_t));
                rx_data_frame._1_beamer_id = 0;
                rx_data_frame._2_angle_graycode=0;
                rx_data_frame._3_timer_cnt = 0;
                // reset positions
                rx_current_bit_pos = 0;

            }
            /// else - input capture, nothing to do.
            /// TODO: turn off input capture timer when it is not supposed to be used
            break;
        }
            // FIXME: review the state below for usefullness <-


        case RX_DATA_PROCESSNG:
        {
            if(_is_timer_update_event)
            {
                switch(DataFrameState)
                {
                    case DATAFRAME_1_BEAMER_ID:
                    {
                        if(0 == rx_current_bit_pos) // execute only once, when first bit is being processed
                        {
                            // change period only when the very first bit of the whole data frame is being processed
                            up_tim_p->Instance->ARR = DataBitLength;
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

#ifdef DEBUG_READING_DATA_1
                            dbg_pulse_1();
#endif
#ifdef DEBUG_READING_DATA_2
                            dbg_pulse_2();
#endif
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
                            rx_total_bits = sizeof(rx_data_frame._2_angle_graycode) * 8;
                        }
                        // send current bit of current byte
                        if(rx_current_bit_pos < rx_total_bits)  // change to next state
                        {
                            // k-th bit of n: (n >> k) & 1
                            if(is_1_on_update_event())
                            {
                                // set bit at the inversed position
                                rx_data_frame._2_angle_graycode |= 1
                                        << (rx_total_bits - rx_current_bit_pos - 1);
                                        //<< (rx_current_bit_pos);
                            }
                            /// no need to set bit to zero if signal is low, since all bits are initialized to zeros
                            rx_current_bit_pos++;
#ifdef DEBUG_READING_DATA_1
                            dbg_pulse_1();
#endif
#ifdef DEBUG_READING_DATA_2
                            dbg_pulse_2();
#endif
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
                            rx_total_bits = sizeof(rx_data_frame._3_timer_cnt) * 8;
                        }
                        // send current bit of current byte
                        if(rx_current_bit_pos < rx_total_bits)  // change to next state
                        {
                            // k-th bit of n: (n >> k) & 1
                            if(is_1_on_update_event())
                            {
                                // set bit at the inversed position
                                rx_data_frame._3_timer_cnt |= 1
                                        << (rx_total_bits - rx_current_bit_pos - 1);
                                        //<< (rx_current_bit_pos);
                            }
                            /// no need to set bit to zero if signal is low, since all bits are initialized to zeros
                            rx_current_bit_pos++;
#ifdef DEBUG_READING_DATA_1
                            dbg_pulse_1();
#endif
#ifdef DEBUG_READING_DATA_2
                            dbg_pulse_2();
#endif
                        }

                        if(rx_current_bit_pos == rx_total_bits)
                        {
                            /// finished receiving last field. No delay before stop bit sequence (no necessity to wait)
                            /// TODO: need to be on the same page with motionsensor_transmitter.
                            /// At the moment, there is no delay after sending the verry last bit of the data frame

                            // data has been received
                            // TODO: process data buffer when it is to be read from

                            //HAL_TIM_IC_PWM_Start_IT(&htim4); //TODO
                            // reset current bit position
                            rx_current_bit_pos = 0;

                            /// TODO: check skipping the step below?
                            ///ReceiverState = RX_DATA_DONE;
                            ///DataFrameState = DATAFRAME_1_BEAMER_ID;
                            // wait for the end of last data frame bit
                            // e.g.remaining HalfPeriodOfDataBits before [the delay before] stop bit sequence
                            ///up_tim_p->Instance->ARR = HalfDataBitLength;

                            {
                                up_tim_p->Instance->ARR = HalfStartStopHalfDataBitLength;
                                ReceiverState = RX_STOP_BIT_PROCESSING;
                                StartStopSequenceReceiveState = STAGE_OFF0;
                            }
                        }
                        break;
                    }

                }
            }// end of _is_timer_update
            /// else - input capture during data receiving, nothing to do.
            /// TODO: turn off input capture timer when it is not supposed to be used
            break;
        }

        // FIXME: review the state below for usefullness ->
        case RX_DATA_DONE:
        {
            if(_is_timer_update_event)
            {
                // wait for the middle of low level part of stop sequence
                // e.g. half of the delay before the first immpulse of stop bit sequence
                // after the delay
                up_tim_p->Instance->ARR = HalfStartStopBitLength;
                // this is the zero pont for next rising edge IC timing check at STAGE_OFF0_ON1 step
                //ic_tim_p->Instance->CNT = 0;
                ReceiverState = RX_STOP_BIT_PROCESSING;
                StartStopSequenceReceiveState = STAGE_OFF0;
                break;
            }
            /// else - input capture during data receiving, nothing to do.
            /// TODO: turn off input capture timer when it is not supposed to be used
            break;
        }
            // FIXME: review the state below for usefullness <-


        case RX_STOP_BIT_PROCESSING:
        {
            switch (StartStopSequenceReceiveState)
            {
                //low: off confirmation
                case STAGE_OFF0:
                {
                    if(is_0_on_update_event())
                    {
                        // continue reading stop bit sequence with PeriodOfStartStopBits interval
                        up_tim_p->Instance->ARR = StartStopBitLength;
                        StartStopSequenceReceiveState = STAGE_OFF0_ON1;
                        break;
                    }
                    /// Don't need to reset here
                    break;
                }
                //high rising edge: STAGE_OFF1 -> STAGE_ON1
                case STAGE_OFF0_ON1:
                {
                    // rising edge input capture
                    if(is_0_to_1_edge())
                    {
                        // first rising edge don't have previous rising edge for checking timing
                        // TODO: add relaxed timing check, measure from previous
                        // low level confirmation on timer update event
                        //if(is_first_rising_edge_timing_ok())
                        {
                            StartStopSequenceReceiveState = STAGE_ON1;
                            break;
                        }
                    }
                    reset_receiver_state();
                    break;
                }
                //high confirmation
                case STAGE_ON1:
                {
                    if(is_1_on_update_event())
                    {
                        StartStopSequenceReceiveState = STAGE_ON1_OFF1;
                        break;
                    }
                    reset_receiver_state();
                    break;
                }
                //low falling edge: STAGE_ON1 -> STAGE_OFF1
                case STAGE_ON1_OFF1:
                {
                    if(is_1_to_0_edge())
                    {
                        if(is_1_to_0_edge_timing_ok())
                        {
                            StartStopSequenceReceiveState = STAGE_OFF1;
                            break;
                        }
                    }
                    reset_receiver_state();
                    break;
                }
                // low: confirmation
                case STAGE_OFF1:
                {
                    if(is_0_on_update_event())
                    {
                        StartStopSequenceReceiveState = STAGE_OFF1_ON2;
                        break;
                    }
                    reset_receiver_state();
                    break;
                }
                // high rising edge: STAGE_OFF1 -> STAGE_ON2
                case STAGE_OFF1_ON2:
                {
                    // rising edge input capture
                    if(is_0_to_1_edge())
                    {
                        if(is_0_to_1_edge_timing_ok())
                        {
                             StartStopSequenceReceiveState = STAGE_ON2;
                             break;
                        }
                    }
                    reset_receiver_state(); //TODO: check if it needed
                    break;
                }
                // high confirmation
                case STAGE_ON2:
                {
                    // second pulse confrmation
                    if(is_1_on_update_event())
                    {
                        StartStopSequenceReceiveState = STAGE_ON2_OFF2;
                        break;
                    }
                    reset_receiver_state();
                    break;
                }
                // low falling edge: STAGE_ON2 -> STAGE_OFFF2
                case STAGE_ON2_OFF2:
                {
                    if(is_1_to_0_edge())
                    {
                        if(is_1_to_0_edge_timing_ok())
                        {
                            ReceiverState = RX_STOP_BIT_DONE;
                            StartStopSequenceReceiveState = STAGE_0;
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
            if(is_0_on_update_event())
            {
                // we successfully received data, send corresponding event for listeners to read from the data buffer
                // TODO
                copy_data_frame_to_buffer(&rx_data_frame);
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
    StartStopSequenceReceiveState = STAGE_0;
    DataFrameState = DATAFRAME_1_BEAMER_ID;
    //HAL_TIM_Base_Stop_IT(&htim3);
    up_tim_p->Instance->ARR = DelayCheckingPeriod;
    reset_delay_cnt();
    ic_tim_p->Instance->CNT = 0;
    up_tim_p->Instance->CNT = 0;
}
static inline void reset_delay_cnt()
{
    _delay_counter = 0;
    _is_interframe_delay_long_enough = false;
}
static inline void update_delay_cnt()
{
    if(is_0_on_update_event())
    {
        if(_delay_counter++ > DelayCounterMin)
        {
            _is_interframe_delay_long_enough = true;
        }
    }
    else if(is_1_on_update_event())
    {
        reset_delay_cnt();
    }
}
static inline void copy_data_frame_to_buffer(DataFrame_t* df)
{
    data_frames[arr_index]._2_angle_graycode = df->_2_angle_graycode;
    data_frames[arr_index]._1_beamer_id = df->_1_beamer_id;
    data_frames[arr_index]._3_timer_cnt = df->_3_timer_cnt;
    arr_index++;
    //memcpy(df, rx_data_frame_array, sizeof(rx_data_frame));
}
static inline bool is_1_to_0_edge()
{
    return _is_falling_edge;
    //TODO:FIXME: review
    if(_is_direct_logic)
    {
        return _is_falling_edge;
    }
    return _is_rising_edge;
}
static inline bool is_0_to_1_edge()
{
    return _is_rising_edge;
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
static inline bool is_0_to_1_edge_timing_ok()
{
    // current falling edge happens after Period ticks from previous rising edge
    if(_ccr1 - StartStopBitPeriod  < 0)
    {
        if(StartStopBitPeriod - _ccr1 < max_delta_pwm)
        {
            return true;
        }
    }
    else
    {
        if(_ccr1 - StartStopBitPeriod < max_delta_pwm)
        {
            return true;
        }
    }
    return false;
}
static inline bool is_first_0_to_1_edge_timing_ok()
{
    // current falling edge happens after Period ticks from previous rising edge
    if(_ccr1 - StartStopBitLength  < 0)
    {
        if(StartStopBitLength - _ccr1 < max_delta_pwm)
        {
            return true;
        }
    }
    else
    {
        if(_ccr1 - StartStopBitLength < max_delta_pwm)
        {
            return true;
        }
    }
    return false;
}
static inline bool is_1_to_0_edge_timing_ok()
{
#ifdef DEBUG
dbg[dbg_index++] = _ccr2;
#endif
    // falling edge happens after StartStopBitLength ticks from rising edge
    if(_ccr2 - StartStopBitLength < 0)
    {
//#ifdef DEBUG
//            dbg[dbg_index++] = StartStopBitLength - _ccr2 ;
//#endif
        if(StartStopBitLength - _ccr2 < max_delta_pwm)
        {
            return true;
        }
    }
    else
    {
//#ifdef DEBUG
//dbg[dbg_index++] = _ccr2 - StartStopBitLength;
//#endif
        if(_ccr2 - StartStopBitLength < max_delta_pwm)
        {

            return true;
        }
    }
    return false;
}
static inline bool is_ic_after_interframe_delay()
{
#ifdef DEBUG_DELAY_CHECK_1
        dbg_pulse_1();
#endif
#ifdef DEBUG_DELAY_CHECK_2
        dbg_pulse_2();
#endif
    /*
    if(ccr1 > 2500)
    {
        return true;
    }
    return false;
    */


    return _is_interframe_delay_long_enough;

/*
    if(ccr1 - DelayBetweenDataFrames < 0)
    {
        int delta = DelayBetweenDataFrames - ccr1;
        delay_delta[delaydelta_index++] = delta;
        if(DelayBetweenDataFrames - ccr1 < max_delta_delay)
        {
            return true;
        }
    }
    else
    {
        int delta = ccr1 - DelayBetweenDataFrames;
        delay_delta[delaydelta_index++] = delta;
        if(ccr1 - DelayBetweenDataFrames < max_delta_delay)
        {
            return true;
        }
    }
    return false;
*/
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

