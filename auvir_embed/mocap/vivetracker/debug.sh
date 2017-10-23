#!/bin/bash
echo START BUILDING
rm -fr ./build
mkdir build
cd build
#CMAKE_EXPORT_COMPILE_COMMANDS=ON - for ycm_extra_conf in vim completer plugin
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=DEBUG ..
make

cd ..


#kill hanging openocd if any
ps axf | grep openocd | grep -v grep | awk '{print "kill -9 " $1}' | sh

#start debugger
#(openocd  -f ./openocd.cfg -c "init" -c "reset halt" -c "flash write_image erase vivetracker" -c "reset run")

echo START OPENOCD

(openocd -f ../openocd.cfg &)
#(st-util &)

echo START GDB

arm-none-eabi-gdb ./build/vivetracker -command=../gdb_start_script

#kill hanging openocd if any
ps axf | grep openocd | grep -v grep | awk '{print "kill -9 " $1}' | sh
