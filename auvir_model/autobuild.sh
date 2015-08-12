#!/bin/bash

#
# Watch over project files, and automatically rebuild
# project when files are modified. Can easily be
# customized for different project types/languages.
#
# Use: ./autobuild.sh [executable]
#
# Optional: Pass path to executable to run whenever
#           build succeeds.
#
# e.g.  ./autobuild.sh bin/main
#

commnd='find ./
 -name "[!\.]*.cpp" -or
 -name "[!\.]*.h"   -or
 -name "CMakeLists.txt"
 | xargs stat -c %y | md5sum'

compileCmmnd='mkdir ./build/debug && cd ./build/debug && cmake ../../ -D CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-7.0 && make && cd ../../'

# ----------------------------------------

md5sumStart=`eval $commnd`
clear
echo "Waiting for changes."

while [[ true ]]
do
    md5sumNow=$md5sumStart
    # Loop until some files have changed
    while [[ "$md5sumNow" = "$md5sumStart" ]]
    do
        sleep 0.5
        md5sumNow=`eval $commnd`
    done

    # Recompile
    clear
    #$compileCmmnd
    mkdir -p ./build/debug
    cd ./build/debug
    cmake ../../
    compileOk1=$?
    make
    compileOk2=$?
    cd ../../
    md5sumStart=`eval $commnd`
    # Report build ok, or failure (clear if former)
    if [[ $compileOk1 -eq 0 && $compileOk2 -eq 0 ]]
    then
        clear
        echo -e '[\033[1;32m Build OK\033[0m ]'
    else
        echo -e '[\033[0;31m Build Failed\033[0m ]'
    fi

    # Run executed command.
  #  if [[ $# -gt 0  &&  $compileOk -eq 0 ]]
  #  then
  #      if [[  -e ./$1 ]]
  #      then
  #          ./$1 ^
  #      else
  #          echo -e "[\033[0;31m Executable '$1' was not found!\033[0m ]";
  #      fi
  #  fi
done
