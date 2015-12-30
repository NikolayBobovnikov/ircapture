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
/* USER CODE END Includes */

/*
 * TODO list:
 * beamer hub: detect when new beamer is being connected, assign new ID and send it to the beamer
 * beamer hub: synchronize beamers with each other. send signals to each beamer when it is its turn to beam
 *
 * beamer: get ID, synchhronize time when initialize
 * beamer: start sending data on external signal
 *
 * receiver: signal when buffer with data frames is ready to be read from
 *
 * receiver hub: read from the buffer using spi&dma
 * receiver hub: send data to comp using usb / wifi / sockets
 * 4.
 * /

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
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

enum DataFrameStates
{
    DATAFRAME_0_NODATA,
    DATAFRAME_1_BEAMER_ID,
    DATAFRAME_2_ANGLE,
    DATAFRAME_3_TIME
};
volatile uint8_t DataFrameState = DATAFRAME_0_NODATA;

uint8_t rx_data = 0;
volatile size_t rx_total_bits = 0;
volatile uint8_t rx_current_byte_pos = 0;
volatile uint8_t rx_current_bit_pos = 0;
volatile uint8_t rx_bit = 0;

int level[100] = {0};
int pwm_period[100] = {0};
int pwm_length[100] = {0};
int pwm_index = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

//TODO: specify timer constants
const uint16_t pwm_timer_prescaler = 0;
const uint16_t pwm_timer_period = 950 - 1;
const uint16_t pwm_pulse_width = 475;
const uint16_t envelop_timer_prescaler = 72 - 1;
//TODO: specify timer constants
// values below are numbers of timer ticks
const uint16_t PeriodOfDataBits = 2000 - 1;
const uint16_t PeriodBetweenDataFrames = 5000 - 1;
const uint16_t PeriodOfStartStopBits = 1000 - 1;
const uint16_t HalfPeriodOfStartStopBits = 500 - 1;
const uint16_t HalfPeriodOfDataBits = 1000 - 1;

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

enum TransmitterStates
{
    TX_WAITING_FOR_TRANSMISSION,
    TX_SENDING_START_BIT,
    TX_SENDING_DATA,
    TX_SENDING_STOP_BIT,
    TX_DELAY
};
volatile uint8_t TransmitterState = TX_WAITING_FOR_TRANSMISSION;

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
volatile uint8_t StartStopSequenceTransmitState = STAGE_ON1;
volatile uint8_t StartStopSequenceReceiveState = STAGE_ON1;

const uint8_t max_delta_pwm = 5;
const uint8_t max_delta_pwm_width = 5;

// level 1
void receive_handler();
inline void reset_receiver_state();
inline bool is_rising_edge_in_correct_time();
inline bool is_falling_edge_in_correct_time();
inline bool is_high_level_in_correct_time();
inline bool is_low_level_in_correct_time();
inline bool is_timer_update();
inline bool is_rising_edge();

inline void receive_data_frame_part();
inline void p_w_demodulate(uint8_t bit);
inline void copy_data_frame_to_buffer(DataFrame_t* df);


inline void nop();

HAL_StatusTypeDef HAL_TIM_IC_PWM_Start_IT (TIM_HandleTypeDef *htim);

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //MX_DMA_Init();
  //MX_I2C1_Init();
  //MX_SPI1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */
  //HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
  //HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
  HAL_TIM_IC_PWM_Start_IT(&htim4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
      //send_data();
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

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c1);

}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi1.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi1);

}

/* TIM2 init function */
void MX_TIM2_Init(void)
{

    TIM_ClockConfigTypeDef sClockSourceConfig;
    //TIM_MasterConfigTypeDef sMasterConfig;
    TIM_SlaveConfigTypeDef sSlaveConfig;
    TIM_IC_InitTypeDef sConfigIC;

    htim4.Instance = TIM2;
    htim4.Init.Prescaler = 71;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 65535;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim2);
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);


    HAL_TIM_IC_Init(&htim2);


    /// TIM_TI1_SetConfig
  //  ● Select the active input for TIMx_CCR1: write the CC1S bits to 01 in the TIMx_CCMR1 register (TI1 selected).
      //SET_BIT(htim4.Instance->CCMR1, TIM_CCMR1_CC1S_0);

  //  ● Select the active polarity for TI1FP1 (used both for capture in TIMx_CCR1 and counter clear):
  //    write the CC1P bit to ‘0’ (active on rising edge).
      //SET_BIT(htim4.Instance->CCMR1, TIM_CCER_CC1P)
      sConfigIC.ICFilter = 0;
      sConfigIC.ICPolarity = TIM_ICPOLARITY_RISING;
      sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
      HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1);

  //  ● Select the active input for TIMx_CCR2: write the CC2S bits to 10 in the TIMx_CCMR1  register (TI1 selected).

  //  ● Select the active polarity for TI1FP2 (used for capture in TIMx_CCR2): write the CC2P bit to ‘1’ (active on falling edge).

      sConfigIC.ICFilter = 0;
      sConfigIC.ICPolarity = TIM_ICPOLARITY_FALLING;// TIM_ICPOLARITY_RISING? TODO
      sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
      HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1);//TIM_CHANNEL_2? TODO
  //  ● Select the valid trigger input: write the TS bits to 101 in the TIMx_SMCR register (TI1FP1 selected).
  //  ● Configure the slave mode controller in reset mode: write the SMS bits to 100 in the TIMx_SMCR register.
      sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
      sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
      HAL_TIM_SlaveConfigSynchronization(&htim2, &sSlaveConfig);
  //  ● Enable the captures: write the CC1E and CC2E bits to ‘1’ in the TIMx_CCER register.

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = envelop_timer_prescaler;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = PeriodOfStartStopBits;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

}

/* TIM4 init function */
void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_IC_Init(&htim4);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);

  /// TIM_TI1_SetConfig
//  ● Select the active input for TIMx_CCR1: write the CC1S bits to 01 in the TIMx_CCMR1 register (TI1 selected).
    //SET_BIT(htim4.Instance->CCMR1, TIM_CCMR1_CC1S_0);

//  ● Select the active polarity for TI1FP1 (used both for capture in TIMx_CCR1 and counter clear):
//    write the CC1P bit to ‘0’ (active on rising edge).
    //SET_BIT(htim4.Instance->CCMR1, TIM_CCER_CC1P)
    sConfigIC.ICFilter = 0;
    sConfigIC.ICPolarity = TIM_ICPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1);

//  ● Select the active input for TIMx_CCR2: write the CC2S bits to 10 in the TIMx_CCMR1  register (TI1 selected).
//  ● Select the active polarity for TI1FP2 (used for capture in TIMx_CCR2): write the CC2P bit to ‘1’ (active on falling edge).
    sConfigIC.ICFilter = 0;
    sConfigIC.ICPolarity = TIM_ICPOLARITY_FALLING;// TIM_ICPOLARITY_RISING? TODO
    sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
    HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2);//TIM_CHANNEL_2? TODO
//  ● Select the valid trigger input: write the TS bits to 101 in the TIMx_SMCR register (TI1FP1 selected).
//  ● Configure the slave mode controller in reset mode: write the SMS bits to 100 in the TIMx_SMCR register.
    sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
    sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
    HAL_TIM_SlaveConfigSynchronization(&htim4, &sSlaveConfig);
//  ● Enable the captures: write the CC1E and CC2E bits to ‘1’ in the TIMx_CCER register.


}

HAL_StatusTypeDef HAL_TIM_IC_PWM_Start_IT (TIM_HandleTypeDef *htim)
{
  /* Check the parameters */
  assert_param(IS_TIM_CCX_INSTANCE(htim->Instance, TIM_CHANNEL_1));
  assert_param(IS_TIM_CCX_INSTANCE(htim->Instance, TIM_CHANNEL_2));

  /* Enable the TIM Capture/Compare 1 interrupt */
  __HAL_TIM_ENABLE_IT(htim, TIM_IT_CC1);
  /* Enable the TIM Capture/Compare 2 interrupt */
  __HAL_TIM_ENABLE_IT(htim, TIM_IT_CC2);

  /* Enable the Input Capture channel */
  TIM_CCxChannelCmd(htim->Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
  TIM_CCxChannelCmd(htim->Instance, TIM_CHANNEL_2, TIM_CCx_ENABLE);


  /* Enable the Peripheral */
  __HAL_TIM_ENABLE(htim);

  /* Return function status */
  return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_IC_PWM_Stop_IT (TIM_HandleTypeDef *htim)
{
    assert_param(IS_TIM_CCX_INSTANCE(htim->Instance, TIM_CHANNEL_1));
    assert_param(IS_TIM_CCX_INSTANCE(htim->Instance, TIM_CHANNEL_2));

    __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC1);
    __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC2);

    /* Disable the Input Capture channel */
    TIM_CCxChannelCmd(htim->Instance, TIM_CHANNEL_1, TIM_CCx_DISABLE);
    TIM_CCxChannelCmd(htim->Instance, TIM_CHANNEL_2, TIM_CCx_DISABLE);

    /* Disable the Peripheral */
    __HAL_TIM_DISABLE(htim);

    /* Return function status */
    return HAL_OK;
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
  // GPIO Ports Clock Enable
  __GPIOD_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void receive_handler()
{

    if(pwm_index == 100)
    {
        rx_total_bits  = 0;

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
            if(is_rising_edge())
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
                    if(is_high_level_in_correct_time()) //high level is confirmed, continue reading start sequence
                    {
                        // first point. change timer period to the period between reading start/stop bit values
                        htim3.Instance->ARR = PeriodOfStartStopBits;
                        StartStopSequenceTransmitState = STAGE_ON1_OFF1;
                        break;
                    }
                    reset_receiver_state();
                    break;
                }
                //Low: STAGE_ON1 -> STAGE_OFF1 input capture
                case STAGE_ON1_OFF1:
                {
                    // falling edge should be detected
                    if(is_falling_edge_in_correct_time())
                    {
                        StartStopSequenceTransmitState = STAGE_OFF1;
                        break;
                    }
                    reset_receiver_state();
                    break;
                }
                //Low: STAGE_OFF1 confirmation
                case STAGE_OFF1:
                {
                    // This should be on update
                    if(is_low_level_in_correct_time())
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
                    if(is_rising_edge_in_correct_time())
                    {
                        StartStopSequenceReceiveState = STAGE_ON2;
                        // wait for half a period of startstop bit sequence
                        break;
                    }
                    reset_receiver_state();
                    break;
                }
                //High: STAGE_ON2 confirmation
                case STAGE_ON2:
                {
                    // This should be on update
                    if(is_high_level_in_correct_time())
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
                    if(is_falling_edge_in_correct_time())
                    {
                        StartStopSequenceReceiveState = STAGE_OFF2;
                        // wait for half a period of startstop bit sequence
                        break;
                    }
                    reset_receiver_state();
                    break;
                }
                //Low: STAGE_OFF2 confirmation
                case STAGE_OFF2:
                {
                    if(is_low_level_in_correct_time())
                    {
                        // turn off input capture temporarily,
                        //wait for the beginning of data transmission
                        // HAL_TIM_IC_PWM_Stop_IT(&htim4); TODO
                        StartStopSequenceTransmitState = STAGE_0;
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
            if(is_timer_update())
            {
                ///Prepare to read data frame

                // start reading data bits after the middle of the first pulse,
                // so wait for another HalfPeriodOfDataBits
                htim3.Instance->ARR = HalfPeriodOfDataBits;
                ReceiverState = RX_DATA_PROCESSNG;
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
            if(is_timer_update())
            {
                receive_data_frame_part();
            }// end of is_timer_update()
            /// else - input capture during data receiving, nothing to do.
            /// TODO: turn off input capture timer when it is not supposed to be used
            break;
        }
        case RX_DATA_DONE:
        {
            if(is_timer_update())
            {
                // wait for the middle of low level part of stop sequence
                // e.g. half of the delay before the first immpulse of stop bit sequence
                // after the delay
                //HAL_TIM_IC_PWM_Start_IT(&htim4); TODO
                htim3.Instance->ARR = HalfPeriodOfStartStopBits;
                TransmitterState = RX_STOP_BIT_PROCESSING;
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
                    if(is_low_level_in_correct_time())
                    {
                        // continue reading stop bit sequence with PeriodOfStartStopBits interval
                        htim3.Instance->ARR = PeriodOfStartStopBits;
                        StartStopSequenceReceiveState = STAGE_ON1_OFF1;
                        break;
                    }
                    reset_receiver_state();
                    break;
                }
                //high rising edge: STAGE_OFF1 -> STAGE_ON1
                case STAGE_OFF0_ON1:
                {
                    // rising edge input capture
                    if(is_rising_edge_in_correct_time())
                    {
                        StartStopSequenceTransmitState = STAGE_ON1;
                        break;
                    }
                    reset_receiver_state();
                    break;
                }
                //high confirmation
                case STAGE_ON1:
                {
                    if(is_high_level_in_correct_time())
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
                    if(is_falling_edge_in_correct_time())
                    {
                        StartStopSequenceReceiveState = STAGE_OFF1;
                        break;
                    }
                    reset_receiver_state();
                    break;
                }
                // low: confirmation
                case STAGE_OFF1:
                {
                    if(is_low_level_in_correct_time())
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
                    if(is_rising_edge_in_correct_time())
                    {
                         StartStopSequenceReceiveState = STAGE_ON2;
                         break;
                    }
                    reset_receiver_state();
                    break;
                }
                // high confirmation
                case STAGE_ON2:
                {
                    // second pulse confrmation
                    if(is_high_level_in_correct_time())
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
                    if(is_falling_edge_in_correct_time())
                    {
                        ReceiverState = RX_STOP_BIT_DONE;
                        StartStopSequenceReceiveState = STAGE_0;
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
            if(is_low_level_in_correct_time())
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
    ReceiverState = RX_WAITING_FOR_START_BIT;
    StartStopSequenceReceiveState = STAGE_0;
    HAL_TIM_Base_Stop_IT(&htim3);
}

bool is_rising_edge_in_correct_time()
{
    if(__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_CC1) != RESET)
    {
        if(__HAL_TIM_GET_IT_SOURCE(&htim4, TIM_IT_CC1) != RESET)
        {
            if(abs(htim4.Instance->CCR1 - PeriodOfStartStopBits) <  max_delta_pwm)
            {
                //TODO: ensure that abs is working
                return true;
            }
        }
    }
    return false;
}
bool is_falling_edge_in_correct_time()
{
    if(__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_CC2) != RESET)
    {
        if(__HAL_TIM_GET_IT_SOURCE(&htim4, TIM_IT_CC2) != RESET)
        {
            if(abs(htim4.Instance->CCR2 - HalfPeriodOfStartStopBits) <  max_delta_pwm)
            {
                return true;
            }
        }
    }
    return false;
}
bool is_high_level_in_correct_time()
{
    if(__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) != RESET)
    {
        if(__HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_UPDATE) != RESET)
        {
            if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET)
            {
                return true;
            }
        }
    }
    return false;
}
bool is_low_level_in_correct_time()
{
    if(__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) != RESET)
    {
        if(__HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_UPDATE) != RESET)
        {
            if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_RESET)
            {
                return true;
            }
        }
    }
    return false;
}
bool is_timer_update()
{
    if(__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) != RESET)
    {
        if(__HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_UPDATE) != RESET)
        {
            return true;
        }
    }
    return false;
}
bool is_rising_edge()
{
    if(__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_CC1) != RESET)
    {
        if(__HAL_TIM_GET_IT_SOURCE(&htim4, TIM_IT_CC1) != RESET)
        {
            return true;
        }
    }
    return false;
}

void receive_data_frame_part()
{
    switch(DataFrameState)
    {
        default:
        {
            reset_receiver_state();
            return;
        }
        case(DATAFRAME_1_BEAMER_ID):
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
            // finished receiving, wait
            else if(rx_current_bit_pos == rx_total_bits)
            {
                // after each field in the data frame, there is a delay with
                // period PeriodOFDataBits, so need to wait, doing nothing in the meantime
                rx_current_bit_pos++;
            }
            // finished waiting, move to next state
            else
            {
                // change state to process second part of the data frame
                DataFrameState = DATAFRAME_2_ANGLE;
                // reset current bit position
                rx_current_bit_pos = 0;
            }
            break;
        }
        case(DATAFRAME_2_ANGLE):
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
            // finished receiving, wait
            else if(rx_current_bit_pos == rx_total_bits)
            {
                // after each field in the data frame, there is a delay with
                // period PeriodOFDataBits, so need to wait, doing nothing in the meantime
                rx_current_bit_pos++;
            }
            // finished waiting, move to next state
            else
            {
                // change state to process second part of the data frame
                DataFrameState = DATAFRAME_3_TIME;
                // reset current bit position
                rx_current_bit_pos = 0;
            }
            break;
        }
        case(DATAFRAME_3_TIME):
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
                ReceiverState = RX_DATA_DONE;
                // wait for the end of data frame
                // e.g.remaining HalfPeriodOfDataBits before [the delay before] stop bit sequence
                htim3.Instance->ARR = HalfPeriodOfDataBits;
                HAL_TIM_Base_Start_IT(&htim3);
                // reset state
                DataFrameState = DATAFRAME_0_NODATA;
                // reset current bit position
                rx_current_bit_pos = 0;
            }
            break;
        }
    }
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
