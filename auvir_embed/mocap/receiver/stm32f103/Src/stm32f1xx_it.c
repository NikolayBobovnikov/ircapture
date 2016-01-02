/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"
// b stm32f1xx_it.c:350

/* USER CODE BEGIN 0 */
#include <stdbool.h>
// TODO: cleanup when done debugging
#define DEBUG



///TODO: refactor constants below
///TODO: learn about typedefs and structs
typedef struct
{
    uint8_t _1_beamer_id;
    uint8_t _2_angle_graycode;
    uint16_t _3_timer_cnt;
} DataFrame_t;
 DataFrame_t rx_data_frame;
#define  RX_BUF_SIZE 10
 DataFrame_t data_frames[RX_BUF_SIZE];
 uint8_t arr_index;

 uint8_t rx_data = 0;
 volatile size_t rx_total_bits = 0;
 volatile uint8_t rx_current_bit_pos = 0;
 volatile uint8_t rx_bit = 0;

//TODO: specify timer constants
extern const uint16_t pwm_timer_prescaler;
extern const uint16_t pwm_timer_period;
extern const uint16_t pwm_pulse_width;
extern const uint16_t envelop_timer_prescaler;
extern const uint16_t DataBitLength;
extern const uint16_t DelayBetweenDataFrames;
extern const uint16_t StartStopBitPeriod;
extern const uint16_t StartStopBitLength;
extern const uint16_t HalfStartStopBitLength;
extern const uint16_t HalfDataBitLength;
extern const uint16_t DelayCheckingPeriod;

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
volatile uint8_t ReceiverState = RX_WAITING_FOR_START_BIT;

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
volatile uint8_t StartStopSequenceReceiveState = STAGE_ON1;

enum DataFrameStates
{
    DATAFRAME_1_BEAMER_ID,
    DATAFRAME_2_ANGLE,
    DATAFRAME_3_TIME,
};
volatile uint8_t DataFrameState = DATAFRAME_1_BEAMER_ID;

enum LineLevels
{
    LINE_UNDEFINED,
    LINE_LOW_ON_UPDATE_EVENT,
    LINE_HIGH_ON_UPDATE_EVENT
};
volatile uint8_t _line_level;

volatile bool _is_rising_edge;
volatile bool _is_falling_edge;
volatile bool _is_timer_update_event;
volatile uint16_t ccr1;
volatile uint16_t ccr2;
volatile uint16_t _delay_counter = 0;

 int level[100] = {0};
 int pwm_period[100] = {0};
 int pwm_length[100] = {0};
 int period_delta[100] = {0};
 int pulse_delta[100] = {0};
 int delay_delta[100] = {0};
 int dbg[100] = {0};
 int level_index=0;
 int pulse_index=0;
 int period_index=0;
 int perioddelta_index=0;
 int pulsedelta_index=0;
 int delaydelta_index=0;
 int dbg_index=0;

const uint8_t max_delta_pwm = 50;
const uint8_t max_delta_pwm_width = 50;
const uint8_t max_delta_delay = 200;
static uint16_t delta;

inline void receive_handler();
inline void reset_receiver_state();
inline bool is_rising_edge_timing_ok();
inline bool is_falling_edge_timing_ok();
inline bool is_ic_after_interframe_delay();
inline void reset_delay_cnt();
inline void copy_data_frame_to_buffer(DataFrame_t* df);


/// For debugging. TODO: cleanup when done
inline void dbg_pulse_A7();
inline void dbg_pulse_A5();

//#define DEBUG_READING_DATA_A5
#define DEBUG_READING_DATA_A7

#define DEBUG_LOW_CHECK_A5
//#define DEBUG_LOW_CHECK_A7

//#define DEBUG_DELAY_CHECK_A5
//#define DEBUG_DELAY_CHECK_A7

//#define DEBUG_DROP_DELAYCNT_A5
//#define DEBUG_DROP_DELAYCNT_A7

//#define DEBUG_RISING_EDGE_A5
//#define DEBUG_RISING_EDGE_A7

//#define DEBUG_FALLING_EDGE_A5
//#define DEBUG_FALLING_EDGE_A7

/* USER CODE END 0 */

/* al variables --------------------------------------------------------*/
 DMA_HandleTypeDef hdma_spi1_rx;
 DMA_HandleTypeDef hdma_spi1_tx;
 SPI_HandleTypeDef hspi1;
 TIM_HandleTypeDef htim1;
 TIM_HandleTypeDef htim2;
 TIM_HandleTypeDef htim3;
 TIM_HandleTypeDef htim4;

/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles DMA1 channel2 global interrupt.
*/
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel3 global interrupt.
*/
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET)
    {
        _line_level = LINE_HIGH_ON_UPDATE_EVENT;
        reset_delay_cnt();
    }
    else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_RESET)
    {
        _line_level = LINE_LOW_ON_UPDATE_EVENT;

        _delay_counter++;


#ifdef DEBUG_LOW_CHECK_A7
        dbg_pulse_A7();
#endif
#ifdef DEBUG_LOW_CHECK_A5
        dbg_pulse_A5();
#endif
#ifdef DEBUG_DROP_DELAYCNT_A5
        dbg_pulse_A5();
#endif
#ifdef DEBUG_DROP_DELAYCNT_A7
        dbg_pulse_A7();
#endif
    }
    else
    {
        _line_level = LINE_UNDEFINED;
         reset_delay_cnt();
        return;
    }
    _is_timer_update_event = true;
    _is_rising_edge = false;
    _is_falling_edge = false;

    /* USER CODE BEGIN TIM3_IRQn 0 */
    receive_handler();
    _is_timer_update_event = false;
    /* USER CODE END TIM3_IRQn 0 */
    HAL_TIM_IRQHandler(&htim3);
    /* USER CODE BEGIN TIM3_IRQn 1 */
    /* USER CODE END TIM3_IRQn 1 */
}

/**
* @brief This function handles TIM4 global interrupt.
*/
void TIM4_IRQHandler(void)
{
    if(__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_CC1) != RESET)
    {
        if(__HAL_TIM_GET_IT_SOURCE(&htim4, TIM_IT_CC1) != RESET)
        {
            // workaround for reset in slave mode not working?
            htim4.Instance->CNT=0;
            ccr1 = htim4.Instance->CCR1;
            _is_rising_edge = true;
            _is_falling_edge = false;

#ifdef DEBUG_RISING_EDGE_A7
            dbg_pulse_A7();
#endif
#ifdef DEBUG_RISING_EDGE_A5
            dbg_pulse_A5();
#endif
            if(period_index < 100)
            {
                pwm_period[period_index++] = ccr1;
            }
        }

    }
    else if(__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_CC2) != RESET)
    {
        if(__HAL_TIM_GET_IT_SOURCE(&htim4, TIM_IT_CC2) != RESET)
        {
            if(pulse_index < 100)
            {
                pwm_length[pulse_index++] = ccr2;
            }
            ccr2 = htim4.Instance->CCR2;
            _is_falling_edge = true;
            _is_rising_edge = false;

#ifdef DEBUG_FALLING_EDGE_A7
            dbg_pulse_A7();
#endif
#ifdef DEBUG_FALLING_EDGE_A5
            dbg_pulse_A5();
#endif
        }
    }
    _is_timer_update_event = false;
    _line_level = LINE_UNDEFINED;

    receive_handler();
    // reset helper vars
    _is_rising_edge = false;
    _is_falling_edge = false;


  /* USER CODE BEGIN TIM4_IRQn 0 */


  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
  /* USER CODE END TIM4_IRQn 1 */
}

/**
* @brief This function handles SPI1 global interrupt.
*/
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */

  /* USER CODE END SPI1_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi1);
  /* USER CODE BEGIN SPI1_IRQn 1 */

  /* USER CODE END SPI1_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void receive_handler()
{
    if(period_index == 100)
    {
        period_index  = 0;
    }
    if(pulse_index == 100)
    {
        pulse_index  = 0;
    }
    if(perioddelta_index == 100)
    {
        perioddelta_index  = 0;
    }
    if(pulsedelta_index == 100)
    {
        pulsedelta_index  = 0;
    }
    if(delaydelta_index == 100)
    {
        delaydelta_index  = 0;
    }
    if(dbg_index == 100)
    {
        dbg_index  = 0;
    }
    if(arr_index == RX_BUF_SIZE)
    {
        //data_frames
        int a = 0;
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

    /* 1. Waiting for start bit
     * 2. Input capture channel: rising edge detected. Start timer, wait 1/2 of period
     * 3. 1/2 period check: if signal is high, start bit is received, goto 4, otherwise stop timer, goto 1
     * 4. 1/2 + i period check: read timer signal, increment received bits count
     * 5. If rbc == total_bits, wait for stop bit
     * 6. Check stop bit: if signal is high, stop timer, save received data, update event (?), goto 1
     *
     */
    switch(ReceiverState)
    {
        case RX_WAITING_FOR_START_BIT:
        {
            // wait for delay DelayBetweenDataFrames ticks
            // before rising edge

            // This should be on IC event
            if(_is_rising_edge )
            {
                if(is_ic_after_interframe_delay()) //TODO: check delay before data frame
                {
                    // wait for first point
                    //HAL_TIM_Base_Start_IT(&htim3);
                    htim3.Instance->CNT = 0;

                    htim3.Instance->ARR = HalfStartStopBitLength;
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
                    if(LINE_HIGH_ON_UPDATE_EVENT == _line_level)
                    {
                        // first point. change timer period to the period between reading start/stop bit values
                        htim3.Instance->ARR = StartStopBitLength;
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
                    if(_is_falling_edge)
                    {
                        if(is_falling_edge_timing_ok())
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
                    if(LINE_LOW_ON_UPDATE_EVENT == _line_level)
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
                    if(_is_rising_edge)
                    {
                        if(is_rising_edge_timing_ok())
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
                    if(LINE_HIGH_ON_UPDATE_EVENT == _line_level)
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
                    if(_is_falling_edge)
                    {
                        if(is_falling_edge_timing_ok())
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
                    if(LINE_LOW_ON_UPDATE_EVENT == _line_level)
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
                    if(_is_rising_edge)
                    {
                        if(is_rising_edge_timing_ok())
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
                    if(LINE_HIGH_ON_UPDATE_EVENT == _line_level)
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
                    if(_is_falling_edge)
                    {
                        if(is_falling_edge_timing_ok())
                        {
                            StartStopSequenceReceiveState = STAGE_OFF4;
                            // restart a counter to reduce integrating of error,
                            // wait for half a period of startstop bit sequence
                            htim3.Instance->CNT = 0;
                            htim3.Instance->ARR = HalfStartStopBitLength;
                            break;
                        }
                    }
                    reset_receiver_state();
                    break;
                }
                //Low: STAGE_OFF2 confirmation
                case STAGE_OFF4:
                {
                    if(LINE_LOW_ON_UPDATE_EVENT == _line_level)
                    {
                        // turn off input capture temporarily,
                        //wait for the beginning of data transmission
                        //HAL_TIM_IC_PWM_Stop_IT(&htim4); //TODO
                        StartStopSequenceReceiveState = STAGE_0;
                        ReceiverState = RX_START_BIT_DONE;
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
        // Transitiolal state, adjust timer period so that we start readind data bits in the middle of each signal
        case RX_START_BIT_DONE:
        {
            if(_is_timer_update_event)
            {
                ///Prepare to read data frame

                // start reading data bits after the middle of the first pulse,
                // so wait for another HalfPeriodOfDataBits
                htim3.Instance->ARR = HalfDataBitLength;
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
                            htim3.Instance->ARR = DataBitLength;
                            rx_total_bits = sizeof(rx_data_frame._1_beamer_id) * 8;
                        }
                        // send current bit of current byte
                        if(rx_current_bit_pos < rx_total_bits)  // change to next state
                        {
                            // k-th bit of n: (n >> k) & 1
                            if(LINE_HIGH_ON_UPDATE_EVENT == _line_level)
                            {
                                // set bit at the inversed position
                                rx_data_frame._1_beamer_id |= 1
                                        << (rx_total_bits - rx_current_bit_pos - 1);
                                        //<< (rx_current_bit_pos);
                            }
                            /// no need to set bit to zero if signal is low, since all bits are initialized to zeros
                            rx_current_bit_pos++;

#ifdef DEBUG_READING_DATA_A5
                            dbg_pulse_A5();
#endif
#ifdef DEBUG_READING_DATA_A7
                            dbg_pulse_A7();
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
                            if(LINE_HIGH_ON_UPDATE_EVENT == _line_level)
                            {
                                // set bit at the inversed position
                                rx_data_frame._2_angle_graycode |= 1
                                        << (rx_total_bits - rx_current_bit_pos - 1);
                                        //<< (rx_current_bit_pos);
                            }
                            /// no need to set bit to zero if signal is low, since all bits are initialized to zeros
                            rx_current_bit_pos++;
#ifdef DEBUG_READING_DATA_A5
                            dbg_pulse_A5();
#endif
#ifdef DEBUG_READING_DATA_A7
                            dbg_pulse_A7();
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
                            if(LINE_HIGH_ON_UPDATE_EVENT == _line_level)
                            {
                                // set bit at the inversed position
                                rx_data_frame._3_timer_cnt |= 1
                                        << (rx_total_bits - rx_current_bit_pos - 1);
                                        //<< (rx_current_bit_pos);
                            }
                            /// no need to set bit to zero if signal is low, since all bits are initialized to zeros
                            rx_current_bit_pos++;
#ifdef DEBUG_READING_DATA_A5
                            dbg_pulse_A5();
#endif
#ifdef DEBUG_READING_DATA_A7
                            dbg_pulse_A7();
#endif
                        }
                        /// finished receiving last field. No delay before stop bit sequence (no necessity to wait)
                        /// TODO: need to be on the same page with motionsensor_transmitter.
                        /// At the moment, there is no delay after sending the verry last bit of the data frame
                        if(rx_current_bit_pos == rx_total_bits - 1)
                        {
                            // data has been received
                            // TODO: process data buffer
                            //rx_data_frame
                            ReceiverState = RX_DATA_DONE;
                            DataFrameState = DATAFRAME_1_BEAMER_ID;
                            // wait for the end of last data frame bit
                            // e.g.remaining HalfPeriodOfDataBits before [the delay before] stop bit sequence
                            htim3.Instance->ARR = HalfDataBitLength;

                            //HAL_TIM_IC_PWM_Start_IT(&htim4); //TODO
                            // reset current bit position
                            rx_current_bit_pos = 0;
                        }
                        break;
                    }

                }
            }// end of _is_timer_update
            /// else - input capture during data receiving, nothing to do.
            /// TODO: turn off input capture timer when it is not supposed to be used
            break;
        }
        case RX_DATA_DONE:
        {
            if(_is_timer_update_event)
            {
                // wait for the middle of low level part of stop sequence
                // e.g. half of the delay before the first immpulse of stop bit sequence
                // after the delay
                htim3.Instance->ARR = HalfStartStopBitLength;
                ReceiverState = RX_STOP_BIT_PROCESSING;
                StartStopSequenceReceiveState = STAGE_OFF0;
                break;
            }
            /// else - input capture during data receiving, nothing to do.
            /// TODO: turn off input capture timer when it is not supposed to be used
            break;
        }
        case RX_STOP_BIT_PROCESSING:
        {
            switch (StartStopSequenceReceiveState)
            {
                //low: off confirmation
                case STAGE_OFF0:
                {
                    if(LINE_LOW_ON_UPDATE_EVENT == _line_level)
                    {
                        // continue reading stop bit sequence with PeriodOfStartStopBits interval
                        htim3.Instance->ARR = StartStopBitLength;
                        StartStopSequenceReceiveState = STAGE_OFF0_ON1;
                        break;
                    }
                    reset_receiver_state();
                    break;
                }
                //high rising edge: STAGE_OFF1 -> STAGE_ON1
                case STAGE_OFF0_ON1:
                {
                    // rising edge input capture
                    if(_is_rising_edge)
                    {
                        // first rising edge don't have previous rising edge for checking timing
                        // TODO: add relaxed timing check, measure from previous
                        // low level confirmation on timer update event
                        //if(is_rising_edge_timing_ok())
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
                    if(LINE_HIGH_ON_UPDATE_EVENT == _line_level)
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
                    if(_is_falling_edge)
                    {
                        if(is_falling_edge_timing_ok())
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
                    if(LINE_LOW_ON_UPDATE_EVENT == _line_level)
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
                    if(_is_rising_edge)
                    {
                        dbg[dbg_index++]=ccr1;

                        if(is_rising_edge_timing_ok())
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
                    if(LINE_HIGH_ON_UPDATE_EVENT == _line_level)
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
                    if(_is_falling_edge)
                    {
                        if(is_falling_edge_timing_ok())
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
            if(LINE_LOW_ON_UPDATE_EVENT == _line_level)
            {
                // we successfully received data, send corresponding event for listeners to read from the data buffer
                // TODO
                copy_data_frame_to_buffer(&rx_data_frame);
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
void reset_receiver_state()
{
    //HAL_TIM_IC_PWM_Start_IT(&htim4); //TODO
    ReceiverState = RX_WAITING_FOR_START_BIT;
    StartStopSequenceReceiveState = STAGE_0;
    DataFrameState = DATAFRAME_1_BEAMER_ID;
    //HAL_TIM_Base_Stop_IT(&htim3);
    htim3.Instance->ARR = DelayCheckingPeriod;
    reset_delay_cnt();
    htim4.Instance->CNT = 0;
}
bool is_rising_edge_timing_ok()
{
    // current falling edge happens after Period ticks from previous rising edge
    if(ccr1 - StartStopBitPeriod  < 0)
    {
        int delta = StartStopBitPeriod - ccr1;
        period_delta[perioddelta_index++] = delta;
        if(StartStopBitPeriod - ccr1 < max_delta_pwm)
        {
            return true;
        }
    }
    else
    {
        int delta = ccr1 - StartStopBitPeriod;
        period_delta[perioddelta_index++] = delta;
        if(ccr1 - StartStopBitPeriod < max_delta_pwm)
        {
            return true;
        }
    }
    return false;
}
bool is_falling_edge_timing_ok()
{
    // falling edge happens after StartStopBitLength ticks from rising edge
    if(ccr2 - StartStopBitLength < 0)
    {
        int delta = StartStopBitLength - ccr2;
        pulse_delta[pulsedelta_index++] = delta;
        if(StartStopBitLength - ccr2 < max_delta_pwm)
        {
            return true;
        }
    }
    else
    {
        int delta = ccr2 - StartStopBitLength;
        pulse_delta[pulsedelta_index++] = delta;
        if(ccr2 - StartStopBitLength < max_delta_pwm)
        {
            return true;
        }
    }
    return false;
}
bool is_ic_after_interframe_delay()
{
#ifdef DEBUG_DELAY_CHECK_A5
        dbg_pulse_A5();
#endif
#ifdef DEBUG_DELAY_CHECK_A7
        dbg_pulse_A7();
#endif
    /*
    if(ccr1 > 2500)
    {
        return true;
    }
    return false;
    */

    if(DelayCheckingPeriod == htim3.Instance->ARR  ) // &&_delay_counter > DelayBetweenDataFrames - 100 // && _delay_counter < 60
    {
        if( _delay_counter > DelayBetweenDataFrames)
        {
            delta = _delay_counter - DelayBetweenDataFrames;

        }
        else
        {
            delta = DelayBetweenDataFrames - _delay_counter;;
        }

        if(delaydelta_index < 100)
        {
            {
                delay_delta[delaydelta_index++] = delta;

            }
        }

        if(delta < max_delta_delay)
        {
            return true;
        }

    }

    return true;
    //return false;

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
void reset_delay_cnt()
{
    _delay_counter = 0;

#ifdef DEBUG_DROP_DELAYCNT_A5
        dbg_pulse_A5();
#endif
#ifdef DEBUG_DROP_DELAYCNT_A7
        dbg_pulse_A();
#endif
}
void copy_data_frame_to_buffer(DataFrame_t* df)
{
    data_frames[arr_index]._2_angle_graycode = df->_2_angle_graycode;
    data_frames[arr_index]._1_beamer_id = df->_1_beamer_id;
    data_frames[arr_index]._3_timer_cnt = df->_3_timer_cnt;
    arr_index++;
    //memcpy(df, rx_data_frame_array, sizeof(rx_data_frame));
}
void dbg_pulse_A7()
{
#ifdef DEBUG
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_RESET);
#endif//DEBUG
}
void dbg_pulse_A5()
{
#ifdef DEBUG
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_RESET);
#endif
}


/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
