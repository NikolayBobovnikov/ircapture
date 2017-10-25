#ifndef H_COMMON
#define H_COMMON

#include "stm32f1xx_hal.h"
#include <stdbool.h>

#define DELAY_PRESCALER (72 - 1)

// pin mapping for radio module
#ifdef USE_OLD_MAPPING
#define USE_NEW_MAPPING (!USE_OLD_MAPPING)
#else
#define USE_NEW_MAPPING 1
#endif

#if USE_NEW_MAPPING
#include "pin_mapping_new.h"
#else
#include "pin_mapping_old.h"
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

class GPIO_PIN {
public:
  GPIO_PIN(GPIO_TypeDef *port, uint16_t pin) : _Port(port), _Pin(pin) {}

  GPIO_TypeDef *Port() { return _Port; }
  uint16_t Pin() { return _Pin; }

private:
  GPIO_TypeDef *_Port;
  uint16_t _Pin;
};

void delay_us(uint16_t delay);
void delay_cycles(uint16_t delay);
void delay_timer_general(uint16_t prescaler, uint16_t delay);

void blink(uint8_t num_blinks, uint16_t delay_ms);
void blink_gpiopin(GPIO_PIN pin, uint8_t num_blinks, uint16_t delay_ms);
void blink_port_pin(GPIO_TypeDef *Port, uint16_t Pin, uint8_t num_blinks,
                    uint16_t delay_ms);

#endif // H_COMMON
