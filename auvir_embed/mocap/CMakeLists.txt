cmake_minimum_required(VERSION 2.6)

if(UNIX)
SET(TOOLCHAIN_PREFIX /usr)
endif(UNIX)

if(WIN32)
SET(TOOLCHAIN_PREFIX D:/Software/arm_gcc/6-2017-q2-update)
endif(WIN32)

project (rootproject)
 
add_subdirectory(vivetracker)
add_subdirectory(motionsensor_receiver)
add_subdirectory(motionsensor_transmitter)
add_subdirectory(motionsensor_usbdevice)


ExternalProject_Add(vivetracker
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}/project_a
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
)
