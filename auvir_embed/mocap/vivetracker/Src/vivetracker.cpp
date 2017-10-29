#include "vivetracker.h"
#include <cstring>
#include "se8r01.h"

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
GPIO_PIN ViveTracker::my_led_pin = GPIO_PIN_OUTPUT(GPIOA, GPIO_PIN_0);
I2C_HandleTypeDef ViveTracker::_handle_i2c2;
SPI_HandleTypeDef ViveTracker::_handle_spi1;
TIM_HandleTypeDef ViveTracker::_handle_tim1;
TIM_HandleTypeDef ViveTracker::_handle_tim2;
TIM_HandleTypeDef ViveTracker::_handle_tim3;
TIM_HandleTypeDef ViveTracker::_handle_tim4;

namespace auvir {
extern TIM_HandleTypeDef *const phtim_delay = &htim2;
}

// main global object
static ViveTracker vivetracker(hi2c2, hspi1, htim1, htim2, htim3, htim4);

void nop() {}

void my_test() {
  int buf_src[10] = {1};
  int buf_dst[10] = {0};

  std::memcpy(buf_dst, buf_src, 10);

  nop();
}

const ReturnValue &ViveTracker::init_imu() { return _result; }

const ReturnValue &ViveTracker::init_radio() {
  // configure_gpio_radio();
  setup_radio(&default_module, NRF24_Mode::Receiver);

  return _result;
}

//=================================
// Vivetracker C API
extern "C" {

// inilailize everything
void vivetracker_initialize() {
  // TODO
  // GPIO_PIN_OUTPUT my_led_pin(GPIOA, GPIO_PIN_0);
}

// vivetracker main loop
void vivetracker_loop() {
  // TODO
  ViveTracker::my_led_pin.Toggle();
  // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);

  HAL_Delay(100);
}
}
