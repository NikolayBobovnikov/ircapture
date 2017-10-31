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
// todo: define it somewhere else?
#define USING_SECOND_RADIO_MODULE 1
#ifdef USING_SECOND_RADIO_MODULE
#define NRF24_CSN2_Pin GPIO_PIN_4
#define NRF24_CSN2_Port GPIOA

#define NRF24_CE2_Pin GPIO_PIN_5
#define NRF24_CE2_Port GPIOA

#define NRF24_IRQ2_Pin GPIO_PIN_12
#define NRF24_IRQ2_Port GPIOB

// pins for MPU6050 module
// MPU6050_ADDRESS_AD0
#define MPU6050_AD0_Pin GPIO_PIN_1
#define MPU6050_AD0_Port GPIOA

#endif

// LED pins for debugging and nice blinking
#define LED_ONBOARD_Pin GPIO_PIN_13
#define LED_ONBOARD_Port GPIOC

#define LED_DBG_Pin GPIO_PIN_0
#define LED_DBG_Port GPIOA

// PWM generation: move from TIM4 to TIM2 CH1 (pin A0, check it)

// CSN  C15
// CE   A0
// IRQ  C14

#endif // PING_MAPPING_NEW_H
