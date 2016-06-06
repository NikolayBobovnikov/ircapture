#ifndef common_h
#define common_h

#include "common.h"

extern TIM_HandleTypeDef htim2;

void delay_us(uint16_t delay)
{
    htim2.Instance->CNT = 0;
    htim2.Instance->ARR = (DELAY_PRESCALER + 1) * (delay) - 1;
    __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
    while(__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) == RESET){}
}

void delay_cycles(uint16_t delay)
{
    while(--delay > 0);
}


#endif



void blink(uint8_t num_blinks, uint16_t delay_ms)
{
    for(uint8_t iteration = 0; iteration < num_blinks; ++iteration){
        HAL_GPIO_WritePin(LED_ONBOARD_Port, LED_ONBOARD_Pin, HIGH);
        HAL_Delay(delay_ms);
        HAL_GPIO_WritePin(LED_ONBOARD_Port, LED_ONBOARD_Pin, LOW);
        HAL_Delay(delay_ms);
    }
}
