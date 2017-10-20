#TODO: autodetect arm gcc
#TODO: SET(TOOLCHAIN_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/../../toolchain/gcc-arm-none-eabi-4_9-2015q2)
if(UNIX)
SET(TOOLCHAIN_PREFIX /usr)
endif(UNIX)

if(WIN32)
SET(TOOLCHAIN_PREFIX D:/Software/arm_gcc/6-2017-q2-update)
endif(WIN32)

SET(TARGET_TRIPLET arm-none-eabi)
SET(STM32_CHIP STM32F103C8)
SET(STM32_FAMILY F1)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/../cmake/)
SET(CMAKE_TOOLCHAIN_FILE ${CMAKE_MODULE_PATH}/gcc_stm32.cmake)

#=======================================================================================================
SET(CMAKE_CURRENT_BINARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build)

PROJECT(motionsensor_receiver)
CMAKE_MINIMUM_REQUIRED(VERSION 3.0)
MESSAGE(STATUS "Project: " ${PROJECT_NAME} "\nBuild dir: " ${CMAKE_CURRENT_BINARY_DIR})
ENABLE_LANGUAGE(ASM)

SET(CMAKE_EXPORT_COMPILE_COMMANDS ON)
#TODO: SET to RELEASE in production
message(STATUS "Setting build type to debug")
SET(CMAKE_BUILD_TYPE DEBUG)

# for cmake-ide in emacs: save project dir/build dir to dir-locals.el
#TODO: check with rtags and cmake-ide
message(STATUS "create .dir-locals.el")
file(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/.dir-locals.el
"((nil . (
  (cmake-ide-project-dir . \"${CMAKE_CURRENT_SOURCE_DIR}/\")
  (cmake-ide-build-dir . \"${PROJECT_BINARY_DIR}/\")
 ))
 ((c-mode . ((mode . c++))))
)")

## Preprocessor definitions. required by STM library.
ADD_DEFINITIONS(
  -DSTM32F103xB
  #-DSTM32F10X_MD_VL
  -DUSE_HAL_DRIVER
  #-DUSE_STDPERIPH_DRIVER
  -D__arm__
)
#=======================================================================================================

SET(THIRDPARTY_STM32LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../Libraries)
SET(I2CDevLib_STM32 ${THIRDPARTY_STM32LIB_PATH}/i2cdevlib_hal)
SET(radio_lib_dir ${THIRDPARTY_STM32LIB_PATH}/radio)
SET(I2CDevLib_MPU6050 ${I2CDevLib_STM32}/MPU6050)
SET(I2CDevLib_I2Cdev ${I2CDevLib_STM32}/I2Cdev)
SET(COMMON_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../common)

SET(SRC_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Src
    ${I2CDevLib_MPU6050}
    ${I2CDevLib_I2Cdev}
    ${radio_lib_dir}
    ${COMMON_SRC_DIR}
)


include(../cmake/CollectSources.cmake)
