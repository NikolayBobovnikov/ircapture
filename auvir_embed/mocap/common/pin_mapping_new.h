#ifndef PING_MAPPING_NEW_H
#define PING_MAPPING_NEW_H

/*

NRF24:
A8   OUT  CSN
A9   OUT  CE
A10  IN   IRQ

MPU6050:
A4   OUT  MPU5060_AD0
A5   IN   MPU5060_INT

LED:
A0   OUT  LED_DBG
C13  OUT  LED_ONBOARD

Input capture:
A15  tim2
B3   tim2
A2   tim2
A3   tim2
A6   tim3
A7   tim3
B0   tim3
B1   tim3
B6   tim4
B7   tim4
B8   tim4
B9   tim4

I2C:
B10 SCL
B11 SDA

SPI:
B13 SCK
B14 MISO
B15 MOSI

USB:
A11
A12

SWD Debugging:
A13
*/

// pins for NRF24 module#1
/*
 * CSN A8 OUT
 * CE  A9 OUT
 * IRQ A10 IN
 */
#define NRF24_CSN1_Pin GPIO_PIN_8
#define NRF24_CSN1_Port GPIOA

#define NRF24_CE1_Pin GPIO_PIN_9
#define NRF24_CE1_Port GPIOA

#define NRF24_IRQ1_Pin GPIO_PIN_10
#define NRF24_IRQ1_Port GPIOA

// pins for MPU6050 module
// MPU6050_ADDRESS_AD0
/*
 * A4 OUT
 * A5 IN
 */
#define MPU6050_AD0_Pin GPIO_PIN_4
#define MPU6050_AD0_Port GPIOA

#define MPU6050_INT_Pin GPIO_PIN_5
#define MPU6050_INT_Port GPIOA
#endif

// LED pins for debugging and nice blinking
/*
 * C13 OUT
 */
#define LED_ONBOARD_Pin GPIO_PIN_13
#define LED_ONBOARD_Port GPIOC

/*
 * A0 OUT
 */
#define LED_DBG_Pin GPIO_PIN_0
#define LED_DBG_Port GPIOA

// FOR HUB/USB device only
// pins for NRF24 module#2 (optional, for usbdevice which is using both modules)
// todo: define it somewhere else?
/*
 * CSN A4
 * CE  A5
 * IRQ B12
 */
#define USING_SECOND_RADIO_MODULE 1
#ifdef USING_SECOND_RADIO_MODULE
#define NRF24_CSN2_Pin GPIO_PIN_4
#define NRF24_CSN2_Port GPIOA

#define NRF24_CE2_Pin GPIO_PIN_5
#define NRF24_CE2_Port GPIOA

#define NRF24_IRQ2_Pin GPIO_PIN_12
#define NRF24_IRQ2_Port GPIOB

// PWM generation: move from TIM4 to TIM2 CH1 (pin A0, check it)

#endif // PING_MAPPING_NEW_H
