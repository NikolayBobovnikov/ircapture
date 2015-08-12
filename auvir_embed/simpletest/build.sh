cd ./build
rm -fr ./*
cmake -DCMAKE_TOOLCHAIN_FILE=arm_toolchain.cmake ../
