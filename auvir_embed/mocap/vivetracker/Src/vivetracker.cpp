#include "gpio.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"

#include <cstring>
#include "se8r01.h"
#include "vivetracker.h"

// radio
extern NRF_Module default_module;

// peripherals
extern I2C_HandleTypeDef hi2c2;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

// static Vivetracker members
I2C_HandleTypeDef ViveTracker::_handle_i2c2;
SPI_HandleTypeDef ViveTracker::_handle_spi1;
TIM_HandleTypeDef ViveTracker::_handle_tim1;
TIM_HandleTypeDef ViveTracker::_handle_tim2;
TIM_HandleTypeDef ViveTracker::_handle_tim3;
TIM_HandleTypeDef ViveTracker::_handle_tim4;

namespace auvir {
extern TIM_HandleTypeDef *const phtim_delay = &htim2;
}

void nop() {}

const ReturnValue &ViveTracker::init_imu() { return _result; }

const ReturnValue &ViveTracker::init_radio() {
  // configure_gpio_radio();
  setup_radio(&default_module, NRF24_Mode::Receiver);

  return _result;
}

//=================================
// Vivetracker C API
extern "C" {

// vivetracker main loop
void vivetracker_loop() {
  // main global object
  ViveTracker vivetracker(hi2c2, hspi1, htim1, htim2, htim3, htim4,
                          LED_DBG_Port, LED_DBG_Pin);

  while (true) {
    // TODO
    vivetracker.my_led_pin.Toggle();
    HAL_Delay(1000);
  }
}

}  // extern "C"
