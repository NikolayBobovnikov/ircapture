#option(TOOLCHAIN_PREFIX "/home/nikolay/work/dev/auvir/auvir_embed/toolchain/gcc-arm-none-eabi-4_9-2015q2 -DTARGET_TRIPLET=arm-none-eabi")
#option(TARGET_TRIPLET "arm-none-eabi")
#option(STM32_CHIP "STM32F103C8T6")
#option(CMAKE_TOOLCHAIN_FILE "/home/nikolay/work/dev/auvir/auvir_embed/stm32-cmake/cmake/gcc_stm32.cmake")


set(TOOLCHAIN_PREFIX ${CMAKE_CURRENT_LIST_DIR}/../../toolchain/gcc-arm-none-eabi-4_9-2015q2)
set(THIRDPARTY_STM32LIB_PATH ${CMAKE_CURRENT_LIST_DIR}/../../../../thirdparty/baremetal/stm32)

set(TARGET_TRIPLET arm-none-eabi)
set(STM32_CHIP STM32F103C8)
set(STM32_FAMILY F1)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_LIST_DIR}/../cmake)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_MODULE_PATH}/gcc_stm32.cmake)
set(STM32Cube_DIR ${THIRDPARTY_STM32LIB_PATH}/STM32Cube_FW_${STM32_FAMILY}_V1.1.0)
set(STD_PERIPH_LIBRARY_DIR ${THIRDPARTY_STM32LIB_PATH}/STM32F10x_StdPeriph_Lib_V3.5.0)

