#option(TOOLCHAIN_PREFIX "/home/nikolay/work/dev/auvir/auvir_embed/toolchain/gcc-arm-none-eabi-4_9-2015q2 -DTARGET_TRIPLET=arm-none-eabi")
#option(TARGET_TRIPLET "arm-none-eabi")
#option(STM32_CHIP "STM32F103C8T6")
#option(CMAKE_TOOLCHAIN_FILE "/home/nikolay/work/dev/auvir/auvir_embed/stm32-cmake/cmake/gcc_stm32.cmake")


set(TOOLCHAIN_PREFIX /home/nikolay/work/dev/auvir/auvir_embed/toolchain/gcc-arm-none-eabi-4_9-2015q2 -DTARGET_TRIPLET=arm-none-eabi)
set(TARGET_TRIPLET arm-none-eabi)
set(STM32_CHIP STM32F103C8)
set(STM32_FAMILY F1)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /home/nikolay/work/dev/auvir/auvir_embed/stm32-cmake/cmake)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_MODULE_PATH}/gcc_stm32.cmake)


MESSAGE(STATUS "test message from BuildOptions: STM32_CHIP: ${STM32_CHIP}")
