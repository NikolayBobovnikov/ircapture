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
// TODO: cleanup when done debugging
#define DEBUG
/* USER CODE END Includes */

/*
 * TODO list:
 * receiver: handle inverse signal
 * finish with sending/receiving using IR channel
 * test distibruted channel transmitter (2 timers with separate channels)
 * make beamer  synchronization - start sending on external trigger

 *
 * beamer hub: detect when new beamer is being connected, assign new ID and send it to the beamer
 * beamer hub: synchronize beamers with each other. send signals to each beamer when it is its turn to beam
 *
 * beamer: get ID, synchhronize time when initialize
 * beamer: start sending data on external signal
 *
 * receiver: signal when buffer with data frames is ready to be read from
 * receiver: catch signal from several IR sensors
 *
 * receiver hub: read from the buffer using spi&dma
 * receiver hub: send data to comp using usb / wifi / sockets
 * receiver hub: allocate an array of items for each sensor;
 *              Each item should contain sensor's ID and the time it was inactive
 *              When time_inactive exceeds certain max value, sensor is considered
 *              disconnected? Or send a connection request to it and if no responce - then consider it
 *              disconnected.
 *              When sensor is disconnected, free according item in the array
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
/// parameters for receiver ===================
const GPIO_TypeDef * GPIO_PORT_IR_IN = GPIOB;
const uint16_t GPIO_PIN_IR_IN = GPIO_PIN_6;
const TIM_HandleTypeDef* ic_tim_p = &htim4;
const TIM_HandleTypeDef* up_tim_p = &htim3;
const bool _is_direct_logic = true;
/// ===========================================

// TODO: cleanup?
extern const uint16_t envelop_timer_prescaler;
extern const uint16_t DelayCheckingPeriod;
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
HAL_StatusTypeDef HAL_TIM_IC_PWM_Start_IT (TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_IC_PWM_Stop_IT (TIM_HandleTypeDef *htim);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
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
  MX_TIM3_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(up_tim_p);
  HAL_TIM_IC_PWM_Start_IT(ic_tim_p);
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
  htim3.Init.Period = DelayCheckingPeriod;
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
  htim4.Init.Prescaler = envelop_timer_prescaler;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_IC_Init(&htim4);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);

  /// TIM_TI1_SetConfig
//  ● Select the active input for TIMx_CCR1: write the CC1S bits to 01 in the TIMx_CCMR1 register (TI1 selected).
    //SET_BIT(htim4.Instance->CCMR1, TIM_CCMR1_CC1S_0);

//  ● Select the active polarity for TI1FP1 (used both for capture in TIMx_CCR1 and counter clear):
//    write the CC1P bit to ‘0’ (active on rising edge).
    //SET_BIT(htim4.Instance->CCMR1, TIM_CCER_CC1P)
    sConfigIC.ICFilter = 0;
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

//  ● Select the active input for TIMx_CCR2: write the CC2S bits to 10 in the TIMx_CCMR1  register (TI1 selected).
//  ● Select the active polarity for TI1FP2 (used for capture in TIMx_CCR2): write the CC2P bit to ‘1’ (active on falling edge).
    sConfigIC.ICFilter = 0;
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
//  ● Select the valid trigger input: write the TS bits to 101 in the TIMx_SMCR register (TI1FP1 selected).
//  ● Configure the slave mode controller in reset mode: write the SMS bits to 100 in the TIMx_SMCR register.

    sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
    sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
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


//  ● Enable the captures: write the CC1E and CC2E bits to ‘1’ in the TIMx_CCER register.


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

  GPIO_InitTypeDef GPIO_InitStruct;

#ifdef DEBUG
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif
}

/* USER CODE BEGIN 4 */
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
