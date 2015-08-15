include (CMakeForceCompiler)

set (TOOLCHAIN_PATH /home/nikolay/work/dev/auvir/auvir_embed/toolchain/gcc-arm-none-eabi-4_9-2015q2)

set (TOOLCHAIN_BIN_DIR  ${TOOLCHAIN_PATH}/bin)
set (TOOLCHAIN_LIBC_DIR ${TOOLCHAIN_PATH}/arm-none-eabi/libs)
set (TOOLCHAIN_INC_DIR ${TOOLCHAIN_LIBC_DIR}/include)
set (TOOLCHAIN_LIB_DIR ${TOOLCHAIN_LIBC_DIR}/usr/lib)

set (CMAKE_SYSTEM_NAME LINUX)
set (CMAKE_SYSTEM_PROCESSOR cortex-m3)
set (CMAKE_FIND_ROOT_PATH ${LIBC_DIR})

find_program(GNU_ARM_C arm-none-eabi-gcc ${TOOLCHAIN_BIN_DIR})
find_program(GNU_ARM_CXX arm-none-eabi-g++ ${TOOLCHAIN_BIN_DIR})

CMAKE_FORCE_C_COMPILER(${GNU_ARM_C} GNU)
CMAKE_FORCE_CXX_COMPILER(${GNU_ARM_CXX} GNU)
message(STATUS "gcc:" ${GNU_ARM_C})
#set (CMAKE_C_COMPILER ${GNU_ARM_C} CACHE INTERNAL "")
#set (CMAKE_CXX_COMPILER ${GNU_ARM_CXX} CACHE INTERNAL "" )
