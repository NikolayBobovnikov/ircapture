/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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

/* USER CODE BEGIN Includes */
#include <stdbool.h>
//TODO: cleanup when done debuugging
#define DEBUG
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//PWM timer configuration
TIM_HandleTypeDef * phtim_envelop = &htim1;
const uint16_t pwm_timer_prescaler = 0;
const uint16_t pwm_timer_period = 949;
const uint16_t pwm_pulse_width = 475;
const uint16_t envelop_timer_prescaler = 72 - 1;
//TODO: specify timer constants
// values below are numbers of timer ticks
const uint16_t StartStopBitLength = 1000 - 1;
const uint16_t DataBitLength = 2000 - 1;
const uint16_t DelayBetweenDataFrames = 5000 - 1;


///TODO: refactor constants below
typedef struct
{
    uint8_t _1_beamer_id;
    uint8_t _2_angle_graycode;
    uint16_t _3_timer_cnt;
} DataFrame_t;

DataFrame_t tx_data_frame;
#define  TX_BUF_SIZE 1
volatile size_t tx_total_bits = 0;
volatile uint8_t tx_current_bit_pos = 0;
volatile uint8_t tx_bit = 0;

uint8_t level[100] = {0};
uint16_t pwm_period[100] = {0};
uint8_t arr_index = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

enum ReceiverStates
{
    RX_WAITING_FOR_START_BIT,
    RX_START_BIT_SENDING,
    RX_START_BIT_SENT,
    RX_DATA_RECEIVING,
    RX_DATA_RECEIVED,
    RX_STOP_BIT_RECEIVING,
    RX_STOP_BIT_RECEIVED
};
volatile uint8_t ReceiverState = RX_WAITING_FOR_START_BIT;

enum TransmitterStates
{
    TX_WAITING,
    TX_START_BIT,
    TX_DATA,
    TX_STOP_BIT,
    TX_DELAY
};
volatile uint8_t TransmitterState = TX_WAITING;

enum DataFrameStates
{
    DATAFRAME_0_NODATA,
    DATAFRAME_1_BEAMER_ID,
    DATAFRAME_2_ANGLE,
    DATAFRAME_3_TIME
};
volatile uint8_t DataFrameState = DATAFRAME_0_NODATA;


enum StartStopSequenceStates
{
    STAGE_0,
    STAGE_ON1,
    STAGE_OFF1,
    STAGE_ON2,
    STAGE_OFF2,
    STAGE_ON3,
    STAGE_OFF3
};
volatile uint8_t StartStopSequenceTransmitState = STAGE_0;
volatile uint8_t StartStopSequenceReceiveState = STAGE_0;

/*
  TIM2_CH1 PA0
  TIM2_CH2 PA1
  TIM2_CH3 PA2
  TIM2_CH4 PA3
  TIM3_CH1 PA6
  TIM3_CH2 PA7
  TIM3_CH3 PB0
  TIM3_CH4 PB1
  TIM4_CH1 PB6
  TIM4_CH2 PB7
  TIM4_CH3 PB8
  TIM4_CH4 PB9
*/


enum OutputChannelsStates
{
    Timer2Channel1, // 1  PA0 v
    Timer2Channel2, // 2  PA1
    Timer2Channel3, // 3  PA2 v
    Timer2Channel4, // 4  PA3
    Timer3Channel1, // 5  PA6 v
    Timer3Channel2, // 6  PA7 v
    Timer3Channel3, // 7  PB0
    Timer3Channel4, // 8  PB1 v
    Timer4Channel1, // 9  PB6
    Timer4Channel2, // 10 PB7
    Timer4Channel3, // 11 PB8 - reserved
    Timer4Channel4  // 12 PB9 - without mask
};
//LED on primary output channel works without mask, so used for sending
//start/stop sequence, beamer ID, time
const uint8_t primary_output_channel = Timer4Channel4;
uint8_t currentOutputTimChannel = Timer4Channel4;
uint8_t input_channel_per_message_bit[sizeof(tx_data_frame._2_angle_graycode) * 8] =
    {
        Timer2Channel1, // 1
        Timer2Channel2, // 2
        Timer2Channel3, // 3
        Timer2Channel4, // 4
        Timer3Channel1, // 5
        Timer3Channel2, // 6
        Timer3Channel3, // 7
        Timer3Channel4  // 8
    };


// level 1
void send_data();

// level 2
void transmit_handler();
inline void reset_transmitter();
inline void switch_to_data_transmission_state();
inline void p_w_modulate(uint8_t bit);

// level 3

// level 4
inline void force_envelop_timer_output_on();
inline void force_envelop_timer_output_off();

inline void nop();

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  //  htim_envelop = htim1;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  //MX_I2C2_Init();
  //MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(phtim_envelop); // envelop
  //HAL_TIM_Base_Start_IT(&htim2); // pwm
  //HAL_TIM_Base_Start_IT(&htim3); // pwm
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); // pwm
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
  send_data();
  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

    RCC_OscInitTypeDef RCC_OscInitStruct;
      RCC_ClkInitTypeDef RCC_ClkInitStruct;

      RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
      RCC_OscInitStruct.HSEState = RCC_HSE_ON;
      RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
      RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
      RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
      RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
      HAL_RCC_OscConfig(&RCC_OscInitStruct);

      RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
      RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
      RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
      RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
      RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
      HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

      HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

      HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

      /* SysTick_IRQn interrupt configuration */
      HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM1 init function */
void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = envelop_timer_prescaler;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = StartStopBitLength;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim1);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);
}

/* TIM2 init function */
void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_OC_InitTypeDef sConfigOC;

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = pwm_timer_prescaler;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = pwm_timer_period;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim2);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

    HAL_TIM_PWM_Init(&htim2);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = pwm_pulse_width;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);

    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);

    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{
      TIM_ClockConfigTypeDef sClockSourceConfig;
      TIM_MasterConfigTypeDef sMasterConfig;
      TIM_OC_InitTypeDef sConfigOC;

      htim3.Instance = TIM3;
      htim3.Init.Prescaler = pwm_timer_prescaler;
      htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
      htim3.Init.Period = pwm_timer_period;
      htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
      HAL_TIM_Base_Init(&htim3);

      sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
      HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

      HAL_TIM_PWM_Init(&htim3);

      sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
      sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
      HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

      sConfigOC.OCMode = TIM_OCMODE_PWM1;
      sConfigOC.Pulse = pwm_pulse_width;
      sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
      sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
      HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);

      HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);

      HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);

      HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);

}

/* TIM4 init function */
void MX_TIM4_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_OC_InitTypeDef sConfigOC;

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = pwm_timer_prescaler;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = pwm_timer_period;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim4);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);

    HAL_TIM_PWM_Init(&htim4);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = pwm_pulse_width;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);

    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2);

    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3);

    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4);
}

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOD_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

#ifdef DEBUG
  /// Debug outputs
  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
#endif
}

/* USER CODE BEGIN 4 */
void send_data()
{
    // fill tx_data_frame with data

    // sample data. TODO: use actual one
    tx_data_frame._1_beamer_id = 0b11101110;
    tx_data_frame._2_angle_graycode = 0b11100111;
    tx_data_frame._3_timer_cnt = 0b1100110000110011;

    // just repetition of the same data for now.
    // TODO: send updated time
    if(TX_WAITING == TransmitterState)
    {
        TransmitterState = TX_START_BIT;
    }
    else
    {
        //TODO: handle this case
        // data is still being transmitted. Need to finish previous transmission before starting next one
    }
}
void transmit_handler()
{

    /// ensure carrier is not generating
    force_envelop_timer_output_off();  // stop carrier // TODO: check neseccity

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
            nop();
            break;
        }
        case TX_START_BIT:
        {
            phtim_envelop->Instance->ARR = StartStopBitLength;

            // Start sequence consists of signal sequence {1,0,1}
            switch(StartStopSequenceTransmitState)
            {
                    // High
                case STAGE_0:
                {
                    StartStopSequenceTransmitState = STAGE_ON1;
                    force_envelop_timer_output_on();
                    break;
                }
                    // Low
                case STAGE_ON1:
                {
                    StartStopSequenceTransmitState = STAGE_OFF1;
                    force_envelop_timer_output_off(); //TODO done anyway in timer interrupt handler?
                    break;
                }
                    // High
                case STAGE_OFF1:
                {
                    StartStopSequenceTransmitState = STAGE_ON2;
                    force_envelop_timer_output_on();
                    break;
                }
                //Low
                case STAGE_ON2:
                {
                    StartStopSequenceTransmitState = STAGE_OFF2;
                    force_envelop_timer_output_off();
                    break;
                }
                   //High
                case STAGE_OFF2:
                {
                    StartStopSequenceTransmitState = STAGE_ON3;
                    force_envelop_timer_output_on();
                    break;
                }
                   // Low
                case STAGE_ON3:
                {
                    switch_to_data_transmission_state();
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
                    tx_total_bits = sizeof(tx_data_frame._2_angle_graycode) * 8;
                    tx_bit = (tx_data_frame._2_angle_graycode >> (tx_total_bits - tx_current_bit_pos - 1)) & 1;
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
                    tx_total_bits = sizeof(tx_data_frame._3_timer_cnt) * 8;
                    tx_bit = (tx_data_frame._3_timer_cnt >> (tx_total_bits - tx_current_bit_pos - 1)) & 1;
                    p_w_modulate(tx_bit);
                    // go to next bit.
                    if(tx_current_bit_pos < tx_total_bits)
                    {
                        tx_current_bit_pos++;
                    }
                    else
                    {
                        // change state to finish processing data
                        // sending data using separate PWM channels is finished,
                        // return to default PWM channel
                        currentOutputTimChannel = primary_output_channel;

                        /* Set the Autoreload value for start sequence bits*/
                        phtim_envelop->Instance->ARR = StartStopBitLength;

                        // move on to next stage
                        TransmitterState = TX_STOP_BIT;
                        DataFrameState = DATAFRAME_0_NODATA;
                        tx_current_bit_pos = 0;
                    }
                    break;
                }
            }

            // TODO: check if some errors or other options are possible here?
            break;
        }
        case TX_STOP_BIT:
        {
            // TODO: check if worth to move ARR update to step abore

            // Start sequence consists of signal sequence {1,0,1}
            switch(StartStopSequenceTransmitState)
            {
                case STAGE_0:
                {
                    StartStopSequenceTransmitState = STAGE_ON1;
                    force_envelop_timer_output_on();
                    break;
                }
                case STAGE_ON1:
                {
                    StartStopSequenceTransmitState = STAGE_OFF1;
                    force_envelop_timer_output_off();
                    break;
                }
                case STAGE_OFF1:
                {
                    StartStopSequenceTransmitState = STAGE_ON2;
                    force_envelop_timer_output_on();
                    break;
                }

                // transitional state
                // TODO: check if possible to move to beginning of next state (thus remove delay)
                case STAGE_ON2:
                {
                    phtim_envelop->Instance->ARR = DelayBetweenDataFrames;
                    StartStopSequenceTransmitState = STAGE_0;
                    TransmitterState = TX_DELAY;
                    break;
                }
            }
            break;
        }
        case TX_DELAY:
        {
            reset_transmitter();
            break;
        }
    } // switch(TransmitterState)
}
void reset_transmitter()
{
    TransmitterState = TX_WAITING;
    // TODO: add other steps if needed
    phtim_envelop->Instance->ARR = StartStopBitLength;
    StartStopSequenceTransmitState = STAGE_0;
    DataFrameState = DATAFRAME_0_NODATA;
}

void switch_to_data_transmission_state()
{
    TransmitterState = TX_DATA;
    DataFrameState = DATAFRAME_1_BEAMER_ID;
    StartStopSequenceTransmitState = STAGE_0;
    tx_current_bit_pos = 0;
    force_envelop_timer_output_off();
}

void p_w_modulate(uint8_t bit)
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

void force_envelop_timer_output_on()
{
#ifdef DEBUG
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_SET);
#endif
    // Start pwm timer, depending on timer and channel number
    switch(currentOutputTimChannel)
    {
        case Timer2Channel1:
        {
            HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // carrier
            break;
        }
        case Timer2Channel2:
        {
            HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // carrier
            break;
        }
        case Timer2Channel3:
        {
            HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // carrier
            break;
        }
        case Timer2Channel4:
        {
            HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); // carrier
            break;
        }
        case Timer3Channel1:
        {
            HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // carrier
            break;
        }
        case Timer3Channel2:
        {
            HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // carrier
            break;
        }
        case Timer3Channel3:
        {
            HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // carrier
            break;
        }
        case Timer3Channel4:
        {
            HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // carrier
            break;
        }
        case Timer4Channel1:
        {
            HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); // carrier
            break;
        }
        case Timer4Channel2:
        {
            HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); // carrier
            break;
        }
        case Timer4Channel3:
        {
            HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // carrier
            break;
        }
        case Timer4Channel4:
        {
            HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); // carrier
            break;
        }
    }


}

void force_envelop_timer_output_off()
{
/*
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1); // carrier
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2); // carrier
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3); // carrier
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4); // carrier
        HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1); // carrier
        HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2); // carrier
        HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3); // carrier
        HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4); // carrier
        */
#ifdef DEBUG
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);
#endif
    // Stopt pwm timer, depending on timer and channel number
    switch(currentOutputTimChannel)
    {
        case Timer2Channel1:
        {
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1); // carrier
            break;
        }
        case Timer2Channel2:
        {
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2); // carrier
            break;
        }
        case Timer2Channel3:
        {
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3); // carrier
            break;
        }
        case Timer2Channel4:
        {
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4); // carrier
            break;
        }
        case Timer3Channel1:
        {
            HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1); // carrier
            break;
        }
        case Timer3Channel2:
        {
            HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2); // carrier
            break;
        }
        case Timer3Channel3:
        {
            HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3); // carrier
            break;
        }
        case Timer3Channel4:
        {
            HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4); // carrier
            break;
        }
        case Timer4Channel1:
        {
            HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1); // carrier
            break;
        }
        case Timer4Channel2:
        {
            HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2); // carrier
            break;
        }
        case Timer4Channel3:
        {
            HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3); // carrier
            break;
        }
        case Timer4Channel4:
        {
            HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4); // carrier
            break;
        }
    }
}
void nop(){}
/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
