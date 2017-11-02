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

#if (!defined(MPU6050_AD0_Pin) || !defined(MPU6050_AD0_Port))
#error "MPU6050_AD0 Pin not defined"
#endif

enum class VT_ReturnValue { OK, IMU_init_error, radio_init_error };

class MPU6050Sensor {
public:
  MPU6050Sensor(I2C_HandleTypeDef *hi2c);

  void setAD0(GPIO_STATE state) const;

private:
  const GPIO_PIN_OUTPUT _gpio_MPU6050_AD0;
  static constexpr uint8_t DEVICE_ID = 0x34;
};

/*
 * Setup DBG led pin, according to GPIO port and pin described in pin mapping
 * TODO: when add actions, update descripiton
 */
class ViveTracker {
public:
  ViveTracker(I2C_HandleTypeDef _h_i2c, SPI_HandleTypeDef _h_spi,
              TIM_HandleTypeDef _h_tim1, TIM_HandleTypeDef _h_tim2,
              TIM_HandleTypeDef _h_tim3, TIM_HandleTypeDef _h_tim4)
      : _gpio_led_dbg(LED_DBG_Port, LED_DBG_Pin), _imu_sensor(&_h_i2c) {}

  const VT_ReturnValue &init_imu() const;
  const VT_ReturnValue &init_radio() const;

  void ToggleDebugLED() const { _gpio_led_dbg.Toggle(); }

private:
  const GPIO_PIN_OUTPUT _gpio_led_dbg;
  const MPU6050Sensor _imu_sensor;

  static VT_ReturnValue _result;
};

void nop();

#endif // VIVETRACKER_H
