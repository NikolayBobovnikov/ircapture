#!/bin/bash
echo START BUILDING
rm -fr ./build
mkdir build
cd build
#CMAKE_EXPORT_COMPILE_COMMANDS=ON - for ycm_extra_conf in vim completer plugin
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_TOOLCHAIN_FILE=./cmake ..
make #VERBOSE=1

#stlink_path="/home/nikolay/work/dev/auvir/auvir_embed/toolchain/stlink"
stlink_path="../../toolchain/stlink"

#$stlink_path/st-util --stlinkv1
$stlink_path/st-flash write v1 stm32template.bin  0x08000000
