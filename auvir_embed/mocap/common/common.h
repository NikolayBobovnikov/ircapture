#ifndef H_COMMON
#define H_COMMON

#include "stm32f1xx_hal.h"
#include <stdbool.h>

#define DELAY_PRESCALER (72 - 1)

#ifndef LOW
#define LOW GPIO_PIN_RESET
#endif
#ifndef HIGH
#define HIGH GPIO_PIN_SET
#endif

// pin mapping for radio module
#define USE_NEW_MAPPING (!USE_OLD_MAPPING)
#if USE_OLD_MAPPING
#include "pin_mapping_old.h"
#else
#include "pin_mapping_new.h"
#endif

#define LED_ONBOARD_Pin GPIO_PIN_13
#define LED_ONBOARD_GPIO_Port GPIOC
#define LED_ONBOARD_Port GPIOC

#define LED_DBG_Pin GPIO_PIN_3
#define LED_DBG_GPIO_Port GPIOB

// pin mapping for shift register
//#define ShiftReg_MR_NOT_Pin GPIO_PIN_3
#define ShiftReg_MR_NOT_Port GPIOB

#define ShiftReg_OE_NOT_Pin GPIO_PIN_0
#define ShiftReg_OE_NOT_Port GPIOA

#define ShiftReg_STCP_Pin GPIO_PIN_4
#define ShiftReg_STCP_Port GPIOA

typedef struct GPIO_PIN {
  GPIO_TypeDef *Port;
  uint16_t Pin;
} GPIO_PIN;

#ifdef __cplusplus
extern "C" {
#endif
void delay_us(uint16_t delay);
void delay_cycles(uint16_t delay);
void delay_timer_general(uint16_t prescaler, uint16_t delay);

void blink(uint8_t num_blinks, uint16_t delay_ms);
void blink_gpiopin(GPIO_PIN pin, uint8_t num_blinks, uint16_t delay_ms);
void blink_port_pin(GPIO_TypeDef *Port, uint16_t Pin, uint8_t num_blinks,
                    uint16_t delay_ms);

void configure_gpio_radio();

void configure_gpio_shiftreg();

void shiftreg_send_16bit_data(uint16_t data);
void shiftreg_send_8bit_data(uint8_t data);
#ifdef __cplusplus
}
#endif

#endif // H_COMMON
