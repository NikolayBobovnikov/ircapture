#ifndef common_h
#define common_h

#include "common.h"

extern const bool _debug;
extern TIM_HandleTypeDef htim2;
const bool _is_direct_logic = false;

void  init_gpio_led() {
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

void  debug_init_gpio() {
    if(_debug)
    {
        /* these gpio are used elsewhere
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        */
    }
}

void delay_us(uint16_t delay)
{
    /*
    //htim2.Init.Period = DELAY_PRESCALER;
    htim2.Instance->CNT = 0;
    // TODO: delay - 1 results in a more precise measurements
    // Why?
    htim2.Instance->ARR = delay - 1;
    __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
    while(__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) == RESET){}
    */
    delay_cycles(delay);
}

void delay_cycles(uint16_t delay)
{
    for (uint16_t i = 0; i < delay; ++i);
    //while(--delay > 0);
}

void delay_timer_general(uint16_t prescaler, uint16_t delay)
{
    htim2.Init.Period = prescaler;
    htim2.Instance->CNT = 0;
    // TODO: delay - 1 results in a more precise measurements
    // Why?
    htim2.Instance->ARR = delay - 1;
    __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
    while(__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) == RESET){}
}


#endif


