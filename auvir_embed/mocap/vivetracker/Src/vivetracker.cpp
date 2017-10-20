#include "vivetracker.h"
#include "se8r01.h"
#include <cstring>

extern NRF_Module default_module;
extern TIM_HandleTypeDef htim2;
namespace auvir {
extern TIM_HandleTypeDef *const phtim_delay = &htim2;
}

void nop() {}

void my_test() {
  int buf_src[10] = {1};
  int buf_dst[10] = {0};

  std::memcpy(buf_dst, buf_src, 10);

  nop();
}

const ReturnValue &ViveTracker::init_imu() { return _result; }

const ReturnValue &ViveTracker::init_radio() {
  configure_gpio_radio();
  setup_radio(&default_module, NRF24_Mode::Receiver);

  return _result;
}
