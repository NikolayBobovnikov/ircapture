#TODO: autodetect arm gcc
SET(TOOLCHAIN_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/../../toolchain/gcc-arm-none-eabi-4_9-2015q2)
SET(TARGET_TRIPLET arm-none-eabi)
SET(STM32_CHIP STM32F103C8)
SET(STM32_FAMILY F1)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
SET(CMAKE_TOOLCHAIN_FILE ${CMAKE_MODULE_PATH}/gcc_stm32.cmake)

#=======================================================================================================

PROJECT(motionsensor)
CMAKE_MINIMUM_REQUIRED(VERSION 3.3)
ENABLE_LANGUAGE(ASM)

SET(CMAKE_EXPORT_COMPILE_COMMANDS ON)
#TODO: SET to RELEASE in production
message(STATUS "Setting build type to debug")
SET(CMAKE_BUILD_TYPE DEBUG)

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

SET(I2CDevLib_MPU6050 ${I2CDevLib_STM32}/MPU6050)
SET(I2CDevLib_I2Cdev ${I2CDevLib_STM32}/I2Cdev)

SET(LIB_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Src
    ${I2CDevLib_MPU6050}
    ${I2CDevLib_I2Cdev}
)

# SPL modules
SET(STM32_MISC   misc)
SET(STM32_ADC    stm32f10x_adc)
SET(STM32_BKP    stm32f10x_bkp)
SET(STM32_CAN    stm32f10x_can)
SET(STM32_CEC    stm32f10x_cec)
SET(STM32_CRC    stm32f10x_crc)
SET(STM32_DAC    stm32f10x_dac)
SET(STM32_DBGMCU stm32f10x_dbgmcu)
SET(STM32_DMA    stm32f10x_dma)
SET(STM32_EXTI   stm32f10x_exti)
SET(STM32_FLASH  stm32f10x_flash)
SET(STM32_FSMC   stm32f10x_fsmc)
SET(STM32_GPIO   stm32f10x_gpio)
SET(STM32_I2C    stm32f10x_i2c)
SET(STM32_IWDG   stm32f10x_iwdg)
SET(STM32_PWR    stm32f10x_pwr)
SET(STM32_RCC    stm32f10x_rcc)
SET(STM32_RTC    stm32f10x_rtc)
SET(STM32_SDIO   stm32f10x_sdio)
SET(STM32_SPI    stm32f10x_spi)
SET(STM32_TIM    stm32f10x_tim)
SET(STM32_USART  stm32f10x_usart)
SET(STM32_WWDG   stm32f10x_wwdg)



################################################
#TODO: add/fix functions to include only those files which has to be included (those defined in this file, like SPL modules)
macro(IS_THERE value list )
    foreach (_item ${list})
        message(STATUS "_item: " ${_item})
        message(STATUS "value: " ${value})
        if((${_item} MATCHES ${value}) OR (${_item} EQUAL ${value}))
            message(STATUS "SET result to 1 " )
            SET(${result_} 1)
        endif()
    endforeach()
endmacro(IS_THERE)


function(IS_REQUIRED_FUNC _file _required_src_list _is_required)
    get_filename_component(_filename ${_file} NAME)
    foreach (_required_src ${_required_src_list})
        if((${_file} MATCHES ${_required_src}) OR (${_filename} EQUAL ${_required_src}))
            SET(${_is_required} 1 PARENT_SCOPE)
            message(STATUS "in function: add ${_filename}")
        endif()
    endforeach()
endfunction(IS_REQUIRED_FUNC)

function(IS_INCLUDED_MODULE_FUNC _file _module_list _is_included)
    get_filename_component(_filename ${_file} NAME)
    foreach (_module ${_module_list})
        message(STATUS "check module:  ${_module}")
        if((${_file} MATCHES ${_module}) OR (${_filename} IN_LIST _module_list))
            SET(${_is_included} 1 PARENT_SCOPE)
            message(STATUS "in function: add ${_filename}")
        endif()
    endforeach()
endfunction(IS_INCLUDED_MODULE_FUNC)

####### Recursive addition of library source and header files from all used libraries #######
SET(ALL_LIB_SOURCES "")
SET(ALL_LIB_HEADERS "")

# include source/include files recursively
foreach (_dir ${LIB_DIRS})
    file(GLOB_RECURSE LIB_SOURCES "${_dir}/*.c" "${_dir}/*.s")
    file(GLOB_RECURSE LIB_HEADERS "${_dir}/*.h")
    list(APPEND ALL_LIB_SOURCES ${LIB_SOURCES})
    list(APPEND ALL_LIB_HEADERS ${LIB_HEADERS})
endforeach()

list(REMOVE_DUPLICATES ALL_LIB_HEADERS)
list(REMOVE_DUPLICATES ALL_LIB_SOURCES)


# include directory for every header file
SET (INCLUDE_LIB_DIRS "")
foreach (_headerFile ${ALL_LIB_HEADERS})
    get_filename_component(_dir ${_headerFile} PATH)
    list (APPEND INCLUDE_LIB_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES INCLUDE_LIB_DIRS)

#############################################################################################

include_directories(
${INCLUDE_LIB_DIRS}
)

# Build everything into elf
ADD_EXECUTABLE(
    ${CMAKE_PROJECT_NAME}
    ${ALL_LIB_SOURCES}
    ${ALL_LIB_HEADERS}
    )

STM32_SET_TARGET_PROPERTIES(${CMAKE_PROJECT_NAME})
#STM32_ADD_HEX_BIN_TARGETS(${CMAKE_PROJECT_NAME})
# Convert 'elf' into 'hex' and 'bin'
ADD_CUSTOM_COMMAND(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_OBJCOPY} ARGS -Oihex ${CMAKE_PROJECT_NAME} ${CMAKE_PROJECT_NAME}.hex)
ADD_CUSTOM_COMMAND(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_OBJCOPY} ARGS -Obinary ${CMAKE_PROJECT_NAME} ${CMAKE_PROJECT_NAME}.bin)


