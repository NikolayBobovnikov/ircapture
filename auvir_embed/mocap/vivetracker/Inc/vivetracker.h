#ifndef VIVETRACKER_H
#define VIVETRACKER_H

#include "common.h"
#include "stm32f1xx_hal.h"

#if (!defined(NRF24_CSN1_Pin) || !defined(NRF24_CSN1_Port))
#error "CSN Pin for NRF24 not defined"
#endif

#if (!defined(NRF24_CE1_Pin) || !defined(NRF24_CE1_Port))
#error "CE Pin for NRF24 not defined"
#endif

#if (!defined(NRF24_IRQ1_Pin) || !defined(NRF24_IRQ1_Port))
#error "IRQ Pin for NRF24 not defined"
#endif

#if (!defined(LED_ONBOARD_Pin) || !defined(LED_ONBOARD_Port))
#error "Onboard LED Pin not defined"
#endif

#if (!defined(LED_DBG_Pin) || !defined(LED_DBG_Port))
#error "LED Debug Pin not defined"
#endif

enum class ReturnValue { OK, IMU_init_error, radio_init_error };

class IMU {
 public:
  IMU() {}

 private:
  I2C_HandleTypeDef _h_i2c2;
};

class ViveTracker {
 public:
  ViveTracker(I2C_HandleTypeDef _h_i2c2, SPI_HandleTypeDef _h_spi1,
              TIM_HandleTypeDef _h_tim1, TIM_HandleTypeDef _h_tim2,
              TIM_HandleTypeDef _h_tim3, TIM_HandleTypeDef _h_tim4,
              GPIO_TypeDef *_led_port, uint16_t _led_pin)
      : my_led_pin(_led_port, _led_pin)

  {
    _handle_i2c2 = _h_i2c2;
    _handle_spi1 = _h_spi1;
    _handle_tim1 = _h_tim1;
    _handle_tim2 = _h_tim2;
    _handle_tim3 = _h_tim3;
    _handle_tim4 = _h_tim4;
  }

  const ReturnValue &init_imu();
  const ReturnValue &init_radio();

  GPIO_PIN_OUTPUT my_led_pin;

 private:
  static I2C_HandleTypeDef _handle_i2c2;
  static SPI_HandleTypeDef _handle_spi1;
  static TIM_HandleTypeDef _handle_tim1;
  static TIM_HandleTypeDef _handle_tim2;
  static TIM_HandleTypeDef _handle_tim3;
  static TIM_HandleTypeDef _handle_tim4;

  ReturnValue _result;
};

void nop();

#endif  // VIVETRACKER_H
