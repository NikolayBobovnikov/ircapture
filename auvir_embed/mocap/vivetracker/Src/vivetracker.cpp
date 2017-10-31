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

// peripherals
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

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
  ViveTracker vivetracker(hi2c1, hspi1, htim1, htim2, htim3, htim4);
  VT_ReturnValue result = vivetracker.init_imu();

  while (true) {
    // TODO
    vivetracker.ToggleDebugLED();
    HAL_Delay(1000);
  }
}

} // extern "C"

MPU6050Sensor::MPU6050Sensor()
    : _gpio_MPU6050_AD0(MPU6050_AD0_Port, MPU6050_AD0_Pin) {

  // init i2c dev library
  I2Cdev_hi2c = &hi2c1;
  setAD0(GPIO_STATE::LOW);

  MPU6050_initialize();

  uint8_t new_device_id = 0x34;

  // I2Cdev_readBits(devAddr, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT,
  // MPU6050_WHO_AM_I_LENGTH, buffer, 0);
  uint8_t reg = MPU6050_RA_WHO_AM_I;
  uint8_t data;
  /*
  HAL_I2C_Master_Transmit(I2Cdev_hi2c, MPU6050_ADDRESS_AD0_LOW, &reg, 1, 100);
  if (HAL_I2C_Master_Receive(I2Cdev_hi2c, MPU6050_ADDRESS_AD0_LOW, &data, 1,
                             100) == HAL_OK) {
    int a = 0;
  }
  */

  MPU6050_setDeviceID(new_device_id); //(0b110100, 0x34).

  uint8_t deviceid = MPU6050_getDeviceID();
  if (MPU6050_testConnection()) {
    // TODO
  } else {
    // TODO
  }
}

void MPU6050Sensor::setAD0(GPIO_STATE state) const {
  switch (state) {
  case GPIO_STATE::HIGH: {
    _gpio_MPU6050_AD0.Set(GPIO_STATE::HIGH);
    MPU6050_setAddress(MPU6050_ADDRESS_AD0_HIGH);
    break;
  }
  case GPIO_STATE::LOW: {
    _gpio_MPU6050_AD0.Set(GPIO_STATE::HIGH);
    MPU6050_setAddress(MPU6050_ADDRESS_AD0_LOW);
    break;
  }
  }
}
