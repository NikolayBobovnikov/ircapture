#ifndef PING_MAPPING_NEW_H
#define PING_MAPPING_NEW_H

// pins for NRF24 module#1
#define NRF24_CSN1_Pin GPIO_PIN_11
#define NRF24_CSN1_Port GPIOA

#define NRF24_CE1_Pin GPIO_PIN_10
#define NRF24_CE1_Port GPIOA

#define NRF24_IRQ1_Pin GPIO_PIN_0
#define NRF24_IRQ1_Port GPIOB

// pins for NRF24 module#2 (optional, for usbdevice which is using both modules)
// TODO: define USING_SECOND_RADIO_MODULE
#ifdef USING_SECOND_RADIO_MODULE
#define NRF24_CSN2_Pin
#define NRF24_CSN2_Port

#define NRF24_CE2_Pin
#define NRF24_CE2_Port

#define NRF24_IRQ2_Pin
#define NRF24_IRQ2_Port
#endif

// PWM generation: move from TIM4 to TIM2 CH1 (pin A0, check it)

// CSN  C15
// CE   A0
// IRQ  C14

#endif // PING_MAPPING_NEW_H
