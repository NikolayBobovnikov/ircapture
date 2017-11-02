#include "gpio.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"

#include "MPU6050.h"
#include "se8r01.h"
#include "vivetracker.h"
#include <cstring>

// radio
extern NRF_Module default_module;

// i2c dev lib
extern I2C_HandleTypeDef *I2Cdev_hi2c;

namespace auvir {
extern TIM_HandleTypeDef *const phtim_delay = &htim2;
}

VT_ReturnValue ViveTracker::_result;

void nop() {}

const VT_ReturnValue &ViveTracker::init_imu() const { return _result; }

const VT_ReturnValue &ViveTracker::init_radio() const {
  // configure_gpio_radio();
  setup_radio(&default_module, NRF24_Mode::Receiver);

  return _result;
}

//=================================
// Vivetracker C API
extern "C" {

// vivetracker main loop
void vivetracker_loop() {
  // main object
  ViveTracker vivetracker(hi2c2, hspi2, htim1, htim2, htim3, htim4);
  VT_ReturnValue result = vivetracker.init_imu();

  while (true) {
    // TODO
    vivetracker.ToggleDebugLED();
    HAL_Delay(1000);
  }
}

} // extern "C"

MPU6050Sensor::MPU6050Sensor(I2C_HandleTypeDef *hi2c)
    : _gpio_MPU6050_AD0(MPU6050_AD0_Port, MPU6050_AD0_Pin) {
  // init i2c dev library
  I2Cdev_hi2c = hi2c;
  MPU6050_initialize();

  setAD0(GPIO_STATE::HIGH);
  uint8_t id = 0x34;
  uint8_t deviceid = MPU6050_getDeviceID();

  if (MPU6050_testConnection()) {
    // TODO
  } else {
    // TODO
  }
}

void MPU6050Sensor::setAD0(GPIO_STATE state) const {
  _gpio_MPU6050_AD0.Set(state);
  switch (state) {
  case GPIO_STATE::HIGH: {
    MPU6050_setAddress(MPU6050_ADDRESS_AD0_HIGH);
    break;
  }
  case GPIO_STATE::LOW: {
    MPU6050_setAddress(MPU6050_ADDRESS_AD0_LOW);
    break;
  }
  }
}
