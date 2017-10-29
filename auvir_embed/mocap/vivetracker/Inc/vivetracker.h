#ifndef VIVETRACKER_H
#define VIVETRACKER_H

#include "common.h"
#include "stm32f1xx_hal.h"

enum class ReturnValue { OK, IMU_init_error, radio_init_error };

class ViveTracker {
 public:
  ViveTracker(I2C_HandleTypeDef _h_i2c2, SPI_HandleTypeDef _h_spi1,
              TIM_HandleTypeDef _h_tim1, TIM_HandleTypeDef _h_tim2,
              TIM_HandleTypeDef _h_tim3, TIM_HandleTypeDef _h_tim4) {
    _handle_i2c2 = _h_i2c2;
    _handle_spi1 = _h_spi1;
    _handle_tim1 = _h_tim1;
    _handle_tim2 = _h_tim2;
    _handle_tim3 = _h_tim3;
    _handle_tim4 = _h_tim4;
  }

  const ReturnValue &init_imu();
  const ReturnValue &init_radio();

  static GPIO_PIN my_led_pin;

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
void my_test();

#endif  // VIVETRACKER_H
