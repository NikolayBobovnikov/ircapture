cmake_minimum_required(VERSION 3.2)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/)

include(./cmake/gcc_stm32.cmake)
include(./cmake/gcc_stm32f1.cmake)
include(./cmake/stm32_linker.cmake)

project (rootproject)
 
add_subdirectory(receiver)
add_subdirectory(transmitter)
add_subdirectory(usbdevice)
add_subdirectory(vivetracker)
