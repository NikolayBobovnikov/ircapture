include (CMakeForceCompiler)

set (TOOLCHAIN_PATH ~/work/dev/auvir/auvir_embed/toolchain/gcc-arm-none-eabi-4_9-2015q2)

set (TOOLCHAIN_BIN_DIR  ${TOOLCHAIN_PATH}/bin)
set (TOOLCHAIN_LIBC_DIR ${TOOLCHAIN_PATH}/arm-none-eabi/libs)
set (TOOLCHAIN_INC_DIR ${TOOLCHAIN_LIBC_DIR}/include)
set (TOOLCHAIN_LIB_DIR ${TOOLCHAIN_LIBC_DIR}/usr/lib)

set (CMAKE_SYSTEM_NAME LINUX)

set (CMAKE_SYSTEM_PROCESSOR Arm)

set (CMAKE_FIND_ROOT_PATH ${LIBC_DIR})

set (CMAKE_C_COMPILER ${TOOLCHAIN_BIN_DIR}/arm-none-eabi-gcc CACHE INTERNAL "")
set (CMAKE_CXX_COMPILER ${TOOLCHAIN_BIN_DIR}/arm-none-eabi-g++ CACHE INTERNAL "" )
