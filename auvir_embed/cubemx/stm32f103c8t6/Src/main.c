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
bool received_ir_signal = false;
const uint8_t byte = 0b01000011;
const uint8_t total_bits = 8;
volatile uint8_t current_bit_position = 0;
volatile uint8_t bit = 0;
bool start_bit_sent = false;
bool data_frame_sent = false;

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
const uint16_t PeriodOfStartStopBits = 1000;// timer ticks
const uint16_t PeriodOfDataBits = 2000;// timer ticks

enum ReceiverStates
{
    RX_WAITING_FOR_START_BIT,
    RX_START_BIT_SENDING,
    RX_START_BIT_SENT,
    RX_DATA_SENDING,
    RX_DATA_SENT,
    RX_STOP_BIT_SENDING,
    RX_STOP_BIT_SENT
};
volatile uint8_t ReceiverState = RX_WAITING_FOR_START_BIT;

enum TransmitterStates
{
    TX_WAITING_FOR_TRANSMISSION,
    TX_START_BIT_SENDING,
    TX_START_BIT_SENT,
    TX_DATA_SENDING,
    TX_DATA_SENT,
    TX_STOP_BIT_SENDING,
    TX_STOP_BIT_SENT
};
volatile uint8_t TransmitterState = TX_WAITING_FOR_TRANSMISSION;

enum StartStopSequenceStates
{
    STAGE_0,
    STAGE_ON1,
    STAGE_OFF,
    STAGE_ON2
};
volatile uint8_t StartStopSequenceTransmitState = STAGE_0;
volatile uint8_t StartStopSequenceReceiveState = STAGE_0;

// level 1
void pwm_transmit();
void pwm_receive();

// level 2
void enable_config_timer_carrier();
void enable_config_timer_envelop();
void enable_config_ir_gpio();
void choose_message();
void send_data_frame();
void send_protocol_frame();
void transmit_handler();
void receive_handler();

// level 3
void generate_binary_for_ir_frame();
void convert_ir_frame_to_manchester_format();
void transform_binary_from_msb_to_lsb();
void convert_binary_to_pwm_format();

// level 4
void force_envelop_timer_output_on();
void force_envelop_timer_output_off();
void send_start_stop_sequence();

inline void nop();

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */
  //HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim3);

  //HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1); // envelop
  //HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2); // carrier
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

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
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 949;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim2);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000;
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
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 949;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim4);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim4);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 200;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);

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

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void pwm_transmit()
{

    enable_config_timer_carrier();
    enable_config_timer_envelop();
    enable_config_ir_gpio();
    choose_message();
    send_data_frame();

}
void pwm_receive()
{


}

void enable_config_timer_carrier(){}
void enable_config_timer_envelop(){}
void enable_config_ir_gpio(){}
void choose_message()
{

}
void send_data_frame()
{
    // disable_tim_interrupts
    __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC1);

    generate_binary_for_ir_frame();
    convert_ir_frame_to_manchester_format();

    // enable_tim_interrupts
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC1);
}
void send_protocol_frame()
{
    ///generate_binary_for_ir_frame();
    ///transform_binary_from_msb_to_lsb();
    ///convert_binary_to_pwm_format();

    // TODO: get total bit number
    do
    {
        // k-th bit of n: (n >> k) & 1
        bit = (byte >> current_bit_position) & 1;
        if(bit == 1)
        {
            force_envelop_timer_output_on();

        }
        else // bit == 0
        {
            force_envelop_timer_output_off();

        }

        current_bit_position++;
    } while (current_bit_position < total_bits);

}
void transmit_handler()
{
    /* Send data frame
     * 1. start sequence
     * 2. data
     *  2.1 Coded angle
     *  2.2 Time of emission
     *  2.3 Beamer ID
     * 3. data redundancy (repeated, or use error correction code)
     * 4. stop sequence
     */
    // htim3 forming envelop
    // htim2 generates PWM

    switch(TransmitterState)
    {
        case TX_WAITING_FOR_TRANSMISSION:
        {
            nop();
            break;
        }
        case TX_START_BIT_SENDING:
        {
            // TODO: remove timer period setting below since it shoud be redundant (defaul value shoud be suitable)
            // Set the Autoreload value for start sequence bits
            htim3.Instance->ARR = PeriodOfStartStopBits;

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
                    StartStopSequenceTransmitState = STAGE_OFF;
                    force_envelop_timer_output_off();
                    break;
                }
                case STAGE_OFF:
                {
                    StartStopSequenceTransmitState = STAGE_ON2;
                    force_envelop_timer_output_on();
                    break;
                }

                // transitional state
                // TODO: check if possible to move to beginning of next state (thus remove delay)
                case STAGE_ON2:
                {

                    //reset StartStopSequenceState
                    StartStopSequenceTransmitState = STAGE_0;
                    // move to next state
                    TransmitterState = TX_START_BIT_SENT;
                    break;
                }
            }
            break;
        }
        case TX_START_BIT_SENT:
        {
            /* Set the Autoreload value for data bits*/
            htim3.Instance->ARR = PeriodOfDataBits;

            // prepare to start sending data
            current_bit_position = 0;
            // change to next state
            TransmitterState = TX_DATA_SENDING;
            break;
        }
        case TX_DATA_SENDING:
        {
            if(current_bit_position < total_bits)
            {
                // k-th bit of n: (n >> k) & 1
                bit = (byte >> current_bit_position) & 1;
                if(bit == 1)
                {
                    force_envelop_timer_output_on();
                }
                else // bit == 0
                {
                    force_envelop_timer_output_off();
                }
                current_bit_position++;
            }
            else // change to next state
            {
                // TODO: check if worth to move ARR update here
                TransmitterState = TX_DATA_SENT;
            }
            // TODO: check if some errors or other options are possible here?
            break;
        }

        // transitional state
        case TX_DATA_SENT:
        {
            // Set the Autoreload value for start sequence bits
            htim3.Instance->ARR = PeriodOfStartStopBits;

            // move on to next state
            TransmitterState = TX_STOP_BIT_SENDING;
            break;
        }
        case TX_STOP_BIT_SENDING:
        {
            // TODO: check if worth to move ARR update to step abore
            /* Set the Autoreload value for start sequence bits*/
            htim3.Instance->ARR = PeriodOfStartStopBits;

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
                    StartStopSequenceTransmitState = STAGE_OFF;
                    force_envelop_timer_output_off();
                    break;
                }
                case STAGE_OFF:
                {
                    StartStopSequenceTransmitState = STAGE_ON2;
                    force_envelop_timer_output_on();
                    break;
                }

                // transitional state
                // TODO: check if possible to move to beginning of next state (thus remove delay)
                case STAGE_ON2:
                {

                    //reset StartStopSequenceState
                    StartStopSequenceTransmitState = STAGE_0;
                    // move to next state
                    TransmitterState = TX_STOP_BIT_SENT;
                    break;
                }
            }
            break;
        }
        case TX_STOP_BIT_SENT:
        {
            // TODO: check if it is needed to wait after sending data
            force_envelop_timer_output_off();
            // TODO: probably need wait more?

            // reset to initial state
            TransmitterState = TX_WAITING_FOR_TRANSMISSION;
            break;
        }
    }
}
void receive_handler()
{
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
            if(__HAL_TIM_GET_IT_SOURCE(&htim4, TIM_IT_CC1) != RESET) // rising edge detected
            {
                ReceiverState = RX_START_BIT_SENDING;
                StartStopSequenceReceiveState = STAGE_0;
                // wait for half a period of startstop bit sequence
            }

            break;
        }
        case RX_START_BIT_SENDING:
        {
            // Start sequence consists of signal sequence {1,0,1}
            switch(StartStopSequenceReceiveState)
                {
                case STAGE_0:
                {
                    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET) //current level is high, continue reading start sequence
                    {
                        // move on to the next start sequence state
                        StartStopSequenceTransmitState = STAGE_ON1;
                    }
                    else  // in the start sequence current level should be high, reset the state
                    {
                        ReceiverState = STAGE_ON1;
                    }
                    break;
                }
                case STAGE_ON1:
                {
                    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_RESET) //current level is low, continue reading start sequence
                    {
                        // move on to the next start sequence state
                        StartStopSequenceTransmitState = STAGE_OFF;
                    }
                    else // in the start sequence current level should be low, reset the state
                    {
                        ReceiverState = RX_WAITING_FOR_START_BIT;
                    }
                    break;
                }
                case STAGE_OFF:
                {
                    //if current level is high, reading start sequence is done
                    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET)
                    {
                        // move on to the next start sequence state
                        StartStopSequenceTransmitState = STAGE_ON2;
                    }
                    else // in the start sequence current level should be high, reset the state
                    {
                        ReceiverState = RX_WAITING_FOR_START_BIT;
                    }
                    break;
                }

                // transitional state
                // TODO: check if possible to move to beginning of next state (thus remove delay)
                case STAGE_ON2:
                {
                    //TODO: check that current level is low?
                    //reset StartStopSequenceState
                    StartStopSequenceTransmitState = STAGE_0;
                    // move on to the next receiver state
                    ReceiverState = RX_START_BIT_SENT;
                    break;
                }
            }
            break;
        }
        case RX_START_BIT_SENT:
        {
            //TODO: check that current level is low?

            // change timer period to value corresponding to data bits period
            htim4.Instance->ARR = PeriodOfDataBits;
            break;
        }
        case RX_DATA_SENDING:
        {
            break;
        }
        case RX_DATA_SENT:
        {
            break;
        }
        case RX_STOP_BIT_SENDING:
        {
            break;
        }
        case RX_STOP_BIT_SENT:
        {
            break;
        }
    } // switch(ReceiverState)
}

void generate_binary_for_ir_frame(){}
void convert_ir_frame_to_manchester_format(){}
void transform_binary_from_msb_to_lsb(){}
void convert_binary_to_pwm_format(){}


void force_envelop_timer_output_on()
{
    HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2); // carrier
}
void force_envelop_timer_output_off()
{
    HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_2); // carrier
}
void send_start_stop_sequence()
{

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
