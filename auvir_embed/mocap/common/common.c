#ifndef common_h
#define common_h

#include "common.h"

extern TIM_HandleTypeDef* phtim_delay;
extern SPI_HandleTypeDef hspi1;

void delay_us(uint16_t delay)
{
    phtim_delay->Instance->CNT = 0;
    phtim_delay->Instance->ARR = (DELAY_PRESCALER + 1) * (delay) - 1;
    __HAL_TIM_CLEAR_IT(phtim_delay, TIM_IT_UPDATE);
    while(__HAL_TIM_GET_FLAG(phtim_delay, TIM_FLAG_UPDATE) == RESET){}
}

void delay_cycles(uint16_t delay)
{
    while(--delay > 0);
}


#endif



void blink(uint8_t num_blinks, uint16_t delay_ms)
{
    blink_port_pin(GPIOC, GPIO_PIN_13, num_blinks, delay_ms);
}

void blink_pin(GPIO_PIN pin, uint8_t num_blinks, uint16_t delay_ms)
{
    blink_port_pin(pin.Port, pin.Pin, num_blinks, delay_ms);
}

void blink_port_pin(GPIO_TypeDef *Port, uint16_t Pin, uint8_t num_blinks, uint16_t delay_ms)
{
    for(uint8_t iteration = 0; iteration < num_blinks; ++iteration){
        HAL_GPIO_WritePin(Port, Pin, HIGH);
        HAL_Delay(delay_ms);
        HAL_GPIO_WritePin(Port, Pin, LOW);
        HAL_Delay(delay_ms);
    }
}


void configure_gpio_radio()
{
  GPIO_InitTypeDef GPIO_InitStruct;

  HAL_GPIO_WritePin(NRF24_CSN1_Port, NRF24_CSN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(NRF24_CE1_Port, NRF24_CE1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : NRF24_CSN1_Pin NRF24_CE1_Pin DBG_OUT_1_Pin */
  GPIO_InitStruct.Pin = NRF24_CSN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(NRF24_CSN1_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NRF24_CSN1_Pin NRF24_CE1_Pin DBG_OUT_1_Pin */
  GPIO_InitStruct.Pin = NRF24_CE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(NRF24_CE1_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF24_IRQ1_Pin */
  GPIO_InitStruct.Pin = NRF24_IRQ1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(NRF24_IRQ1_Port, &GPIO_InitStruct);
}

void configure_gpio_shiftreg()
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(ShiftReg_MR_NOT_Port, ShiftReg_MR_NOT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ShiftReg_Expose_Port, ShiftReg_Expose_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ShiftReg_OE_NOT_Port, ShiftReg_OE_NOT_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : ShiftReg_MR_NOT_Pin */
    GPIO_InitStruct.Pin = ShiftReg_MR_NOT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ShiftReg_MR_NOT_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : ShiftReg_OE_NOT_Pin */
    GPIO_InitStruct.Pin = ShiftReg_OE_NOT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ShiftReg_OE_NOT_Port, &GPIO_InitStruct);

    // turn off shift registers
    HAL_GPIO_WritePin(ShiftReg_MR_NOT_Port, ShiftReg_MR_NOT_Pin, HIGH);

    /*Configure GPIO pin : ShiftReg_Expose_Pin */
    GPIO_InitStruct.Pin = ShiftReg_Expose_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ShiftReg_Expose_Port, &GPIO_InitStruct);

    // dont reset shift registers
    HAL_GPIO_WritePin(ShiftReg_MR_NOT_Port, ShiftReg_MR_NOT_Pin, HIGH);

    /*Pin which corresponds to TIM2 CH1 PWM output
    turn on/offf LEDs
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, HIGH);
  */

}

void shiftreg_send_16bit_data(uint16_t data)
{
    uint8_t hi = (uint8_t)(data >> 8);
    uint8_t lo = (uint8_t)(data);
    HAL_GPIO_WritePin(ShiftReg_Expose_Port, ShiftReg_Expose_Pin, LOW);
    HAL_SPI_Transmit(&hspi1, &lo, 1, 10);
    HAL_SPI_Transmit(&hspi1, &hi, 1, 10);
    HAL_GPIO_WritePin(ShiftReg_Expose_Port, ShiftReg_Expose_Pin, HIGH);
    HAL_GPIO_WritePin(ShiftReg_Expose_Port, ShiftReg_Expose_Pin, LOW);

}

void shiftreg_send_8bit_data(uint8_t data)
{
  HAL_GPIO_WritePin(ShiftReg_Expose_Port, ShiftReg_Expose_Pin, LOW);
  HAL_SPI_Transmit(&hspi1, &data, 1, 10);
  // turn off shift registers
  HAL_GPIO_WritePin(ShiftReg_MR_NOT_Port, ShiftReg_MR_NOT_Pin, LOW);
  HAL_GPIO_WritePin(ShiftReg_Expose_Port, ShiftReg_Expose_Pin, HIGH);
  HAL_GPIO_WritePin(ShiftReg_Expose_Port, ShiftReg_Expose_Pin, LOW);

}
