cmake_minimum_required(VERSION 2.8)
project (tests)

set ( THIRDPARTY_LIB_PATH_GTEST "./../../../thirdparty/gtest-1.7.0/" )

#find_package( GTest REQUIRED )
if( GTEST_FOUND )

add_executable (runtests /usr/src/gtest/src/gtest_main.cc ./test.cpp )
target_link_libraries(runtests ${GTEST_BOTH_LIBRARIES})
set( GCC_ADD_LINK_FLAGS "-pthread" )
SET (CMAKE_EXE_LINKER_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_ADD_LINK_FLAGS}")
SET (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCC_ADD_LINK_FLAGS}")

else ( !GTEST_FOUND )

    add_executable (runtests ${THIRDPARTY_LIB_PATH_GTEST}/src/gtest_main.cc ./test.cpp )
    find_library( GTEST_LIBRARY gtest ${THIRDPARTY_LIB_PATH_GTEST}/lib/.libs )
    target_link_libraries(runtests ${GTEST_LIBRARY})

set ( GCC_ADD_LINK_FLAGS "-pthread" )
set ( CMAKE_EXE_LINKER_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_ADD_LINK_FLAGS}" )
#set ( CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCC_ADD_LINK_FLAGS}" )


endif( GTEST_FOUND )
