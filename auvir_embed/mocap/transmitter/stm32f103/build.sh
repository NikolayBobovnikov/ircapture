#!/bin/bash
echo START BUILDING
rm -fr ./build
mkdir build
cd build
#CMAKE_EXPORT_COMPILE_COMMANDS=ON - for ycm_extra_conf in vim completer plugin
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=DEBUG ..
make

#st-flash write v2 stm32test  0x08000000

#kill hanging openocd if any
#ps axf | grep openocd | grep -v grep | awk '{print "kill -9 " $1}' | sh

openocd -f ../openocd.cfg -c "init" -c "reset halt" -c "flash write_image erase motionsensor_transmitter" -c "reset run" -c "shutdown"


#openocd -f interface/stlink-v2.cfg -c "init" -c "reset halt" -c "flash write_image erase motionsensor_transmitter" -c "reset run" -c "shutdown"

#cp stm32* ~/Yandex.Disk/dev/auvir/build

