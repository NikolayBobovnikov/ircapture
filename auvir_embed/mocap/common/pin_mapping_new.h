#ifndef PING_MAPPING_NEW_H
#define PING_MAPPING_NEW_H


#define NRF24_CSN1_Pin GPIO_PIN_15
#define NRF24_CSN1_Port GPIOC

#define NRF24_CE1_Pin GPIO_PIN_0
#define NRF24_CE1_Port GPIOA

#define NRF24_IRQ1_Pin GPIO_PIN_1
#define NRF24_IRQ1_Port GPIOA


#define NRF24_CSN2_Pin GPIO_PIN_2
#define NRF24_CSN2_Port GPIOA

#define NRF24_CE2_Pin GPIO_PIN_3
#define NRF24_CE2_Port GPIOA

#define NRF24_IRQ2_Pin GPIO_PIN_1
#define NRF24_IRQ2_Port GPIOB

// PWM generation: move from TIM4 to TIM2 CH1

// CSN  C15
// CE   A0
// IRQ  C14

#endif //PING_MAPPING_NEW_H
