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
// b stm32f1xx_it.c:640

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
 DataFrame_t recent_data_frames_array[RX_BUF_SIZE];
 uint8_t arr_index;

 uint8_t rx_data = 0;
 volatile size_t rx_total_bits = 0;
 volatile uint8_t rx_current_byte_pos = 0;
 volatile uint8_t rx_current_bit_pos = 0;
 volatile uint8_t rx_bit = 0;

//TODO: specify timer constants
extern const uint16_t pwm_timer_prescaler;
extern const uint16_t pwm_timer_period;
extern const uint16_t pwm_pulse_width;
extern const uint16_t envelop_timer_prescaler;
extern const uint16_t PeriodOfDataBits;
extern const uint16_t PeriodBetweenDataFrames;
extern const uint16_t TwoPeriodsOfStartStopBits;
extern const uint16_t PeriodOfStartStopBits;
extern const uint16_t HalfPeriodOfStartStopBits;
extern const uint16_t HalfPeriodOfDataBits;

enum ReceiverStates
{
    RX_WAITING_FOR_START_BIT,
    RX_START_BIT_PROCESSING,
    RX_START_BIT_DONE,
    RX_DATA_PROCESSNG,
    DATAFRAME_1_BEAMER_ID,
    DATAFRAME_2_ANGLE,
    DATAFRAME_3_TIME,
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
    STAGE_OFF2
};
volatile uint8_t StartStopSequenceReceiveState = STAGE_ON1;

enum LineLevels
{
    LINE_UNDEFINED,
    LINE_LOW,
    LINE_HIGH
};


 int level[100];
 int pwm_period[100];
 int pwm_length[100];
 int period_delta[100];
 int pulse_delta[100];
 int level_index=0;
 int pulse_index=0;
 int period_index=0;
 int perioddelta_index=0;
 int pulsedelta_index=0;

const uint8_t max_delta_pwm = 20;
const uint8_t max_delta_pwm_width = 20;

// level 1
inline void receive_handler();
inline void reset_receiver_state();
inline bool is_period_within_range();
inline bool is_pulse_within_range();
volatile bool _is_rising_edge;
volatile bool _is_falling_edge;
volatile bool _is_timer_update_event;
volatile uint8_t _line_level;
volatile uint16_t ccr1;
volatile uint16_t ccr2;

inline void receive_data_frame_part();
inline void p_w_demodulate(uint8_t bit);
inline void copy_data_frame_to_buffer(DataFrame_t* df);
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
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}


/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET)
    {
        _line_level = LINE_HIGH;
    }
    else
    {
        _line_level = LINE_LOW;
    }
    _is_timer_update_event = true;
    //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_SET);
    //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_RESET);

    /* USER CODE BEGIN TIM3_IRQn 0 */
    receive_handler();
    // reset helper vars
    _is_timer_update_event = false;
    /* USER CODE END TIM3_IRQn 0 */
    HAL_TIM_IRQHandler(&htim3);
    /* USER CODE BEGIN TIM3_IRQn 1 */
    ///HAL_TIM_Base_Stop_IT(&htim3);
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

            if(period_index < 100)
            {
                pwm_period[period_index++] = ccr1;
            }
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_RESET);
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
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_RESET);
        }
    }
    _line_level = LINE_UNDEFINED;
    _is_timer_update_event = false;

    receive_handler();
    // reset helper vars
    _is_rising_edge = false;
    _is_falling_edge = false;

  /* USER CODE BEGIN TIM4_IRQn 0 */


  /* USER CODE END TIM4_IRQn 0 */
  ///HAL_TIM_IRQHandler(&htim4);
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
            // This should be on IC event
            if(_is_rising_edge)
            {
                // wait for first point
                HAL_TIM_Base_Start_IT(&htim3);
                htim3.Instance->ARR = HalfPeriodOfStartStopBits;
                ReceiverState = RX_START_BIT_PROCESSING;
                StartStopSequenceReceiveState = STAGE_ON1;
                // wait for half a period of startstop bit sequence
                break;
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
                    if(LINE_HIGH == _line_level) //high level is confirmed, continue reading start sequence
                    {
                        // first point. change timer period to the period between reading start/stop bit values
                        htim3.Instance->ARR = PeriodOfStartStopBits;
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
                        if(is_pulse_within_range())
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
                    if(LINE_LOW == _line_level)
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
                        if(is_period_within_range())
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
                    if(LINE_HIGH == _line_level)
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
                        if(is_pulse_within_range())
                        {
                            StartStopSequenceReceiveState = STAGE_OFF2;
                            // wait for half a period of startstop bit sequence
                            break;
                        }
                    }
                    reset_receiver_state();
                    break;
                }
                //Low: STAGE_OFF2 confirmation
                case STAGE_OFF2:
                {
                    if(LINE_LOW == _line_level)
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
                htim3.Instance->ARR = HalfPeriodOfDataBits;
                ReceiverState = DATAFRAME_1_BEAMER_ID;
                //ReceiverState = RX_DATA_PROCESSNG;
                //DataFrameState= DATAFRAME_1_BEAMER_ID;
                // initialize buffer with all zeros
                // TODO: fill buffer with zeros
                //memset(&data_frame, 0, sizeof(DataFrame_t));
                rx_data_frame._1_beamer_id = 0;
                rx_data_frame._2_angle_graycode=0;
                rx_data_frame._3_timer_cnt = 0;
                // reset positions
                rx_current_byte_pos = 0;
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
                receive_data_frame_part();
            }// end of _is_timer_update
            /// else - input capture during data receiving, nothing to do.
            /// TODO: turn off input capture timer when it is not supposed to be used
            break;
        }
        case DATAFRAME_1_BEAMER_ID:
        {
            if(_is_timer_update_event)
            {
                if(0 == rx_current_bit_pos) // execute only once, when first bit is being processed
                {
                    // change period only when the very first bit of the whole data frame is being processed
                    htim3.Instance->ARR = PeriodOfDataBits;
                    rx_total_bits = sizeof(rx_data_frame._1_beamer_id) * 8;
                }
                // send current bit of current byte
                if(rx_current_bit_pos < rx_total_bits)  // change to next state
                {
                    // k-th bit of n: (n >> k) & 1
                    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET)
                    {
                        // set bit at the inversed position
                        rx_data_frame._1_beamer_id |= 1 << (rx_total_bits - rx_current_bit_pos - 1);
                    }
                    /// no need to set bit to zero if signal is low, since all bits are initialized to zeros
                    rx_current_bit_pos++;
                }
                // move to next state and wait a delay between data fields
                else
                {
                    // change state to process second part of the data frame
                    ReceiverState = DATAFRAME_2_ANGLE;
                    // reset current bit position
                    rx_current_bit_pos = 0;
                }
            }
            break;
        }
        case DATAFRAME_2_ANGLE:
        {
            if(_is_timer_update_event)
            {
                if(0 == rx_current_bit_pos) // execute only once, when first bit is being processed
                {
                    rx_total_bits = sizeof(rx_data_frame._2_angle_graycode) * 8;
                }
                // send current bit of current byte
                if(rx_current_bit_pos < rx_total_bits)  // change to next state
                {
                    // k-th bit of n: (n >> k) & 1
                    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET)
                    {
                        // set bit at the inversed position
                        rx_data_frame._2_angle_graycode |= 1 << (rx_total_bits - rx_current_bit_pos - 1);
                    }
                    /// no need to set bit to zero if signal is low, since all bits are initialized to zeros
                    rx_current_bit_pos++;
                }
                else
                {
                    // change state to process second part of the data frame
                    ReceiverState = DATAFRAME_3_TIME;
                    // reset current bit position
                    rx_current_bit_pos = 0;
                }
            }
            break;
        }
        case DATAFRAME_3_TIME:
        {
            if(_is_timer_update_event)
            {
                if(0 == rx_current_bit_pos) // execute only once, when first bit is being processed
                {
                    rx_total_bits = sizeof(rx_data_frame._3_timer_cnt) * 8;
                }
                // send current bit of current byte
                if(rx_current_bit_pos < rx_total_bits)  // change to next state
                {
                    // k-th bit of n: (n >> k) & 1
                    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET)
                    {
                        // set bit at the inversed position
                        rx_data_frame._3_timer_cnt |= 1 << (rx_total_bits - rx_current_bit_pos - 1);
                    }
                    /// no need to set bit to zero if signal is low, since all bits are initialized to zeros
                    rx_current_bit_pos++;
                }
                /// finished receiving last field. No necessity to wait
                /// TODO: need to be on the same page with motionsensor_transmitter.
                /// At the moment, there is no delay after sending the verry last bit of the data frame
                else
                {
                    // data has been received
                    // TODO: process data buffer
                    //rx_data_frame
                    ReceiverState = RX_DATA_DONE;
                    // wait for the end of data frame
                    // e.g.remaining HalfPeriodOfDataBits before [the delay before] stop bit sequence
                    htim3.Instance->ARR = HalfPeriodOfDataBits;

                    //HAL_TIM_IC_PWM_Start_IT(&htim4); //TODO
                    // reset current bit position
                    rx_current_bit_pos = 0;
                }
            }
            break;
        }
        case RX_DATA_DONE:
        {
            if(_is_timer_update_event)
            {
                // wait for the middle of low level part of stop sequence
                // e.g. half of the delay before the first immpulse of stop bit sequence
                // after the delay
                //rx_data_frame._1_beamer_id
                htim3.Instance->ARR = HalfPeriodOfStartStopBits;
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
                    if(LINE_LOW == _line_level)
                    {
                        // continue reading stop bit sequence with PeriodOfStartStopBits interval
                        htim3.Instance->ARR = PeriodOfStartStopBits;
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
                        if(is_period_within_range())
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
                    if(LINE_HIGH == _line_level)
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
                        if(is_pulse_within_range())
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
                    if(LINE_LOW == _line_level)
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
                        if(is_period_within_range())
                        {
                             StartStopSequenceReceiveState = STAGE_ON2;
                             break;
                        }
                    }
                    reset_receiver_state();
                    break;
                }
                // high confirmation
                case STAGE_ON2:
                {
                    // second pulse confrmation
                    if(LINE_HIGH == _line_level)
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
                        if(is_pulse_within_range())
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
            if(LINE_LOW == _line_level)
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
void receive_data_frame_part()
{
}
void reset_receiver_state()
{
    //HAL_TIM_IC_PWM_Start_IT(&htim4); //TODO
    ReceiverState = RX_WAITING_FOR_START_BIT;
    StartStopSequenceReceiveState = STAGE_0;
    HAL_TIM_Base_Stop_IT(&htim3);
}
bool is_period_within_range()
{
    if(ccr1 - TwoPeriodsOfStartStopBits  < 0)
    {
        int delta = TwoPeriodsOfStartStopBits - ccr1;
        period_delta[perioddelta_index++] = delta;
        if(TwoPeriodsOfStartStopBits - ccr1 < max_delta_pwm)
        {
            return true;
        }
    }
    else
    {
        int delta = ccr1 - TwoPeriodsOfStartStopBits;
        period_delta[perioddelta_index++] = delta;
        if(ccr1 - TwoPeriodsOfStartStopBits < max_delta_pwm)
        {
            return true;
        }
    }
    return false;
}
bool is_pulse_within_range()
{
    if(ccr2 - PeriodOfStartStopBits < 0)
    {
        int delta = PeriodOfStartStopBits - ccr2;
        pulse_delta[pulsedelta_index++] = delta;
        if(PeriodOfStartStopBits - ccr2 < max_delta_pwm)
        {
            return true;
        }
    }
    else
    {
        int delta = ccr2 - PeriodOfStartStopBits;
        pulse_delta[pulsedelta_index++] = delta;
        if(ccr2 - PeriodOfStartStopBits < max_delta_pwm)
        {
            return true;
        }
    }
    return false;
}
void p_w_demodulate(uint8_t bit)
{

}
void copy_data_frame_to_buffer(DataFrame_t* df)
{
    recent_data_frames_array[arr_index]._2_angle_graycode = df->_2_angle_graycode;
    recent_data_frames_array[arr_index]._1_beamer_id = df->_1_beamer_id;
    recent_data_frames_array[arr_index]._3_timer_cnt = df->_3_timer_cnt;
    //memcpy(df, rx_data_frame_array, sizeof(rx_data_frame));
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
