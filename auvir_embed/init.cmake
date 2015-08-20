set(TOOLCHAIN_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/../../toolchain/gcc-arm-none-eabi-4_9-2015q2)
set(THIRDPARTY_STM32LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../../../thirdparty/baremetal/stm32)
set(TARGET_TRIPLET arm-none-eabi)
set(STM32_CHIP STM32F103C8)
set(STM32_FAMILY F1)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/../cmake)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_MODULE_PATH}/gcc_stm32.cmake)
set(SPL_DIR ${THIRDPARTY_STM32LIB_PATH}/STM32F10x_StdPeriph_Lib_V3.5.0)
#
SET(SPL_INCLUDE_DIRS
    ${SPL_DIR}/Libraries/STM32F10x_StdPeriph_Driver/inc/
    ${SPL_DIR}/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/
    ${SPL_DIR}/Libraries/CMSIS/CM3/CoreSupport/
)


# Startup files
SET(STM32_STARTUP_CL ${SPL_DIR}/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/gcc_ride7/startup_stm32f10x_cl.s)
SET(STM32_STARTUP_HD ${SPL_DIR}/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/gcc_ride7/startup_stm32f10x_hd.s)
SET(STM32_STARTUP_HD_VL ${SPL_DIR}/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/gcc_ride7/startup_stm32f10x_hd_vl.s)
SET(STM32_STARTUP_LD ${SPL_DIR}/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/gcc_ride7/startup_stm32f10x_ld.s)
SET(STM32_STARTUP_LD_VL ${SPL_DIR}/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/gcc_ride7/startup_stm32f10x_ld_vl.s)
SET(STM32_STARTUP_MD ${SPL_DIR}/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/gcc_ride7/startup_stm32f10x_md.s)
SET(STM32_STARTUP_MD_VL ${SPL_DIR}/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/gcc_ride7/startup_stm32f10x_md_vl.s)
SET(STM32_STARTUP_XL ${SPL_DIR}/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/gcc_ride7/startup_stm32f10x_xl.s)

# CMSIS source
SET(STM32_SYSTEM_SOURCE ${SPL_DIR}/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/system_stm32f10x.c)

# SPL modules 
SET(STM32_MISC_SOURCE ${SPL_DIR}/Libraries/STM32F10x_StdPeriph_Driver/src/misc.c)
SET(STM32_ADC_SOURCE ${SPL_DIR}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_adc.c)
SET(STM32_BKP_SOURCE ${SPL_DIR}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_bkp.c)
SET(STM32_CAN_SOURCE ${SPL_DIR}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_can.c)
SET(STM32_CEC_SOURCE ${SPL_DIR}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_cec.c)
SET(STM32_CRC_SOURCE ${SPL_DIR}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_crc.c)
SET(STM32_DAC_SOURCE ${SPL_DIR}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_dac.c)
SET(STM32_DBGMCU_SOURCE ${SPL_DIR}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_dbgmcu.c)
SET(STM32_DMA_SOURCE ${SPL_DIR}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_dma.c)
SET(STM32_EXTI_SOURCE ${SPL_DIR}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_exti.c)
SET(STM32_FLASH_SOURCE ${SPL_DIR}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_flash.c)
SET(STM32_FSMC_SOURCE ${SPL_DIR}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_fsmc.c)
SET(STM32_GPIO_SOURCE ${SPL_DIR}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_gpio.c)
SET(STM32_I2C_SOURCE ${SPL_DIR}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_i2c.c)
SET(STM32_IWDG_SOURCE ${SPL_DIR}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_iwdg.c)
SET(STM32_PWR_SOURCE ${SPL_DIR}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_pwr.c)
SET(STM32_RCC_SOURCE ${SPL_DIR}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_rcc.c)
SET(STM32_RTC_SOURCE ${SPL_DIR}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_rtc.c)
SET(STM32_SDIO_SOURCE ${SPL_DIR}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_sdio.c)
SET(STM32_SPI_SOURCE ${SPL_DIR}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_spi.c)
SET(STM32_TIM_SOURCE ${SPL_DIR}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_tim.c)
SET(STM32_USART_SOURCE ${SPL_DIR}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_usart.c)
SET(STM32_WWDG_SOURCE ${SPL_DIR}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_wwdg.c)

#
