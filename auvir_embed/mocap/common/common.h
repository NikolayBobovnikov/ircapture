#ifndef H_COMMON
#define H_COMMON

#include "stm32f1xx_hal.h"

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

// TODO: configure GPIO_PIN when corresponding object is created

enum class GPIO_MODE {
  INPUT = GPIO_MODE_INPUT,
  OUTPUT_PP = GPIO_MODE_OUTPUT_PP,
  OUTPUT_OD = GPIO_MODE_OUTPUT_OD,
  AF_PP = GPIO_MODE_AF_PP,
  AF_OD = GPIO_MODE_AF_OD,
  AF_INPUT = GPIO_MODE_AF_INPUT
};

enum class GPIO_SPEED {
  FREQ_LOW = GPIO_SPEED_FREQ_LOW,
  FREQ_MEDIUM = GPIO_SPEED_FREQ_MEDIUM,
  FREQ_HIGH = GPIO_SPEED_FREQ_HIGH
};

enum class GPIO_PULL { UP = GPIO_PULLUP, DOWN = GPIO_PULLDOWN };

// creates and configures gpio pin
// TODO: specialize input, output pins
class GPIO_PIN {
 public:
  GPIO_PIN(GPIO_TypeDef *port, uint16_t pin, GPIO_MODE mode, GPIO_SPEED speed,
           GPIO_PULL pull)
      : _Port(port), _Pin(pin) {
    // configure pin and port
    GPIO_InitTypeDef GPIO_InitStruct;
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = static_cast<uint16_t>(mode);
    GPIO_InitStruct.Speed = static_cast<uint16_t>(speed);
    GPIO_InitStruct.Pull = static_cast<uint16_t>(pull);
    HAL_GPIO_Init(port, &GPIO_InitStruct);
  }

  GPIO_TypeDef *Port() { return _Port; }
  uint16_t Pin() { return _Pin; }
  void Toggle() { HAL_GPIO_TogglePin(_Port, _Pin); }

 private:
  GPIO_TypeDef *_Port;
  uint16_t _Pin;
  static GPIO_InitTypeDef GPIO_InitStruct;
};

class GPIO_PIN_INPUT : public GPIO_PIN {
 public:
  GPIO_PIN_INPUT(GPIO_TypeDef *port, uint16_t pin, GPIO_PULL pull)
      : GPIO_PIN(port, pin, GPIO_MODE::INPUT, GPIO_SPEED::FREQ_LOW, pull) {}
};

class GPIO_PIN_OUTPUT : public GPIO_PIN {
 public:
  GPIO_PIN_OUTPUT(GPIO_TypeDef *port, uint16_t pin,
                  GPIO_MODE mode = GPIO_MODE::OUTPUT_PP,
                  GPIO_SPEED speed = GPIO_SPEED::FREQ_LOW)
      : GPIO_PIN(port, pin, mode, speed, GPIO_PULL::DOWN) {}
};

void delay_us(uint16_t delay);
void delay_cycles(uint16_t delay);
void delay_timer_general(uint16_t prescaler, uint16_t delay);

void blink(uint8_t num_blinks, uint16_t delay_ms);
void blink_gpiopin(GPIO_PIN pin, uint8_t num_blinks, uint16_t delay_ms);
void blink_port_pin(GPIO_TypeDef *Port, uint16_t Pin, uint8_t num_blinks,
                    uint16_t delay_ms);

#endif  // H_COMMON
