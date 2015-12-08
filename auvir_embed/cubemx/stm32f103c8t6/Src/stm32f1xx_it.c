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

/* USER CODE BEGIN 0 */
#include <stdbool.h>

int counter = 0;
extern received_ir_signal;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

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
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
  received_ir_signal = true;

  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
//	typedef struct
//	{
//	  __IO uint32_t CR1;             /*!< TIM control register 1,                      Address offset: 0x00 */
//	  __IO uint32_t CR2;             /*!< TIM control register 2,                      Address offset: 0x04 */
//	  __IO uint32_t SMCR;            /*!< TIM slave Mode Control register,             Address offset: 0x08 */
//	  __IO uint32_t DIER;            /*!< TIM DMA/interrupt enable register,           Address offset: 0x0C */
//	  __IO uint32_t SR;              /*!< TIM status register,                         Address offset: 0x10 */
//	  __IO uint32_t EGR;             /*!< TIM event generation register,               Address offset: 0x14 */
//	  __IO uint32_t CCMR1;           /*!< TIM  capture/compare mode register 1,        Address offset: 0x18 */
//	  __IO uint32_t CCMR2;           /*!< TIM  capture/compare mode register 2,        Address offset: 0x1C */
//	  __IO uint32_t CCER;            /*!< TIM capture/compare enable register,         Address offset: 0x20 */
//	  __IO uint32_t CNT;             /*!< TIM counter register,                        Address offset: 0x24 */
//	  __IO uint32_t PSC;             /*!< TIM prescaler register,                      Address offset: 0x28 */
//	  __IO uint32_t ARR;             /*!< TIM auto-reload register,                    Address offset: 0x2C */
//	  __IO uint32_t RCR;             /*!< TIM  repetition counter register,            Address offset: 0x30 */
//	  __IO uint32_t CCR1;            /*!< TIM capture/compare register 1,              Address offset: 0x34 */
//	  __IO uint32_t CCR2;            /*!< TIM capture/compare register 2,              Address offset: 0x38 */
//	  __IO uint32_t CCR3;            /*!< TIM capture/compare register 3,              Address offset: 0x3C */
//	  __IO uint32_t CCR4;            /*!< TIM capture/compare register 4,              Address offset: 0x40 */
//	  __IO uint32_t BDTR;            /*!< TIM break and dead-time register,            Address offset: 0x44 */
//	  __IO uint32_t DCR;             /*!< TIM DMA control register,                    Address offset: 0x48 */
//	  __IO uint32_t DMAR;            /*!< TIM DMA address for full transfer register,  Address offset: 0x4C */
//	  __IO uint32_t OR;              /*!< TIM option register,                         Address offset: 0x50 */
//	}TIM_TypeDef;


  /* USER CODE BEGIN TIM3_IRQn 0 */
  uint32_t CR1    = htim3.Instance->CR1  ;
  uint32_t CR2    = htim3.Instance->CR2  ;
  uint32_t SMCR   = htim3.Instance->SMCR ;
  uint32_t DIER   = htim3.Instance->DIER ;
  uint32_t SR     = htim3.Instance->SR   ;
  uint32_t EGR    = htim3.Instance->EGR  ;
  uint32_t CCMR1  = htim3.Instance->CCMR1;
  uint32_t CCMR2  = htim3.Instance->CCMR2;
  uint32_t CCER   = htim3.Instance->CCER ;
  uint32_t CNT    = htim3.Instance->CNT  ;
  uint32_t PSC    = htim3.Instance->PSC  ;
  uint32_t ARR    = htim3.Instance->ARR  ;
  uint32_t RCR    = htim3.Instance->RCR  ;
  uint32_t CCR1   = htim3.Instance->CCR1 ;
  uint32_t CCR2   = htim3.Instance->CCR2 ;
  uint32_t CCR3   = htim3.Instance->CCR3 ;
  uint32_t CCR4   = htim3.Instance->CCR4 ;
  uint32_t BDTR   = htim3.Instance->BDTR ;
  uint32_t DCR    = htim3.Instance->DCR  ;
  uint32_t DMAR   = htim3.Instance->DMAR ;
  uint32_t OR     = htim3.Instance->OR   ;

  uint32_t cap = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
  __HAL_TIM_SET_COUNTER(&htim2, htim2.Init.Period);
  if(__HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_CC1) == SET)
    {
        int a = 0;
    }
  //if(IS_TIM_OC_MODE(TIM_OCMODE_ACTIVE))

  //htim3.Instance->CCMR1;

  /* USER CODE END TIM3_IRQn 1 */
}

/**
* @brief This function handles TIM4 global interrupt.
*/
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
//	counter = __HAL_TIM_GET_COUNTER(&htim2);

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4); // carrier 38kHz
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

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
