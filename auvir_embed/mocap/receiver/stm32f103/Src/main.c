/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "infrared.h"
#include "sensor.h"
#include "sensor_hub.h"
#include "se8r01.h"

// TODO: cleanup when done debugging
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/// parameters for receiver ===================
TIM_HandleTypeDef* ic_tim_p = &htim4;
TIM_HandleTypeDef* up_tim_p = &htim3;
const GPIO_TypeDef * GPIO_PORT_IR_IN = GPIOB;
const uint16_t GPIO_PIN_IR_IN = GPIO_PIN_6;

TIM_HandleTypeDef* ptim_input_capture = &htim4;
TIM_HandleTypeDef* ptim_data_read = &htim3;

const bool _is_direct_logic = false;
/// ===========================================

uint8_t TX_ADDRESS[TX_ADR_WIDTH]  = {0x10,0x20,0x30,0xab,0xab};
uint8_t rx_buf[TX_PLOAD_WIDTH] = {0}; // initialize value
uint8_t tx_buf[TX_PLOAD_WIDTH] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CRC_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
HAL_StatusTypeDef HAL_TIM_IC_PWM_Start_IT (const TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_IC_PWM_Stop_IT (const TIM_HandleTypeDef *htim);
void send_data_uart();
void nrf24_setup_gpio();
void delay_us(uint8_t us);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/// ======================= USART stuff =======================
HAL_StatusTypeDef status;
extern DataFrame_t rx_data_frame;

enum UART_Commands {
    UART_COMMAND_NOT_RECEIVED = 0,
    UART_DEBUG_DATA_TRANSMIT,
    UART_DEBUG_DATA_TRANSMIT_OK,
    UART_ECHO
};
uint8_t command = UART_COMMAND_NOT_RECEIVED;
uint8_t responce = UART_DEBUG_DATA_TRANSMIT_OK;

typedef struct
{
    uint8_t _ir_hub_id;
    uint8_t _ir_sensor_id;
    DataFrame_t data;
} USART_msg_t;

USART_msg_t uart_msg;

const char mode = 'r'; // 't'
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
    MX_CRC_Init();
    MX_SPI1_Init();
    MX_SPI2_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_USART1_UART_Init();

    /* USER CODE BEGIN 2 */
    debug_init_gpio();
    init_gpio_led();
    nrf24_init();

    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);

    HAL_TIM_Base_Start_IT(ptim_data_read);
    HAL_TIM_IC_PWM_Start_IT(ptim_input_capture);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    // use identical bytes
    bool is_transmitter = (mode =='t');
    bool is_receiver = !is_transmitter;

    setup();

    while (1)
    {
        loop();
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

/* CRC init function */
void MX_CRC_Init(void)
{

    hcrc.Instance = CRC;
    HAL_CRC_Init(&hcrc);

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
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
    hspi1.Init.CRCPolynomial = 7; //10
    HAL_SPI_Init(&hspi1);

}

/* SPI2 init function */
void MX_SPI2_Init(void)
{

    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_SLAVE;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLED;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
    hspi2.Init.CRCPolynomial = 10;
    HAL_SPI_Init(&hspi2);

}

/* TIM2 init function */
void MX_TIM2_Init(void)
{

    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 720 - 1;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim2);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = envelop_timer_prescaler;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = max_period;
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

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);

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
    __GPIOC_CLK_ENABLE();

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

}

/* USER CODE BEGIN 4 */
HAL_StatusTypeDef HAL_TIM_IC_PWM_Start_IT (const TIM_HandleTypeDef *htim)
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
HAL_StatusTypeDef HAL_TIM_IC_PWM_Stop_IT (const TIM_HandleTypeDef *htim)
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
void send_data_uart()
{
    USART_msg_t msg;
    msg._ir_sensor_id = 1;
    msg._ir_hub_id = 2;
    msg.data = rx_data_frame;

    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, (uint8_t *)&msg, sizeof(msg), 10);
    if (status != HAL_OK) {
        // TODO: process error
    }
}

void nrf24_setup_gpio(void) {
    GPIO_InitTypeDef GPIO_InitStruct;

    //Configure IRQ pin
    GPIO_InitStruct.Pin = NRF24_IRQ_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(NRF24_IRQ_PORT, &GPIO_InitStruct);

    //HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
    //HAL_NVIC_EnableIRQ(EXTI4_IRQn);

    //Configure CSN pin
    GPIO_InitStruct.Pin = NRF24_CSN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(NRF24_CSN_PORT, &GPIO_InitStruct);

    //Configure CE pin
    GPIO_InitStruct.Pin = NRF24_CE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(NRF24_CE_PORT, &GPIO_InitStruct);

    /* CSN high = disable SPI */
    HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_SET);

    /* CE low = disable TX/RX */
    HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_RESET);
}

void delay_us(uint8_t delay)
{
    htim2.Instance->CNT = 0;
    HAL_TIM_Base_Start_IT(&htim2);
    while(__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) == RESET)
    {

    }
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
