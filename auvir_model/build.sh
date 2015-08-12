#!/bin/bash

cd ./build
cmake -D CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda  ..
make
