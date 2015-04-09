cmake_minimum_required(VERSION 2.8)

project(auvir_model)

set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)

include_directories( SRC_DIR )


set ( THIRDPARTY_LIB_PATH_OPENCV "./../../thirdparty/opencv/" )

find_package( OpenCV REQUIRED )
if( OPENCV_FOUND )
    message(STATUS "found opencv")
    add_executable (auvir_model main.cpp )
    add_executable(auvir_model ${SRC_DIR}/main.cpp)
    target_link_libraries(auvir_model ${OpenCV_LIBS})

else ( !OPENCV_FOUND )
    message(STATUS "not found opencv")
    #add_executable (auvir_model ${THIRDPARTY_LIB_PATH_OPENCV}  )
    find_library( OpenCV_LIBS gtest ${THIRDPARTY_LIB_PATH_OPENCV} )
    add_executable(auvir_model ${SRC_DIR}/main.cpp )
    target_link_libraries(auvir_model ${OpenCV_LIBS})


endif( OPENCV_FOUND )


set ( Boost_USE_STATIC_LIBS OFF )
set ( Boost_USE_MULTITHREADED ON )
set ( Boost_USE_STATIC_RUNTIME OFF )

set (boost_components filesystem regex)

message (STATUS "boost_components:" ${boost_components})
message (STATUS "MAKE_MODULE_PATH:" ${CMAKE_MODULE_PATH})


#find_package ( BOOST REQUIRED)

