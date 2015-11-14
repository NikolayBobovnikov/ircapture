#!/bin/bash
echo START BUILDING
rm -fr ./build
mkdir build
cd build
#CMAKE_EXPORT_COMPILE_COMMANDS=ON - for ycm_extra_conf in vim completer plugin
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=DEBUG ..
make

cd ..
#start debugger
(openocd -f ./openocd.cfg &)

arm-none-eabi-gdb ./build/stm32test -command=./gdb_start_script
