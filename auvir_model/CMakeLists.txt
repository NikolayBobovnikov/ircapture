cmake_minimum_required(VERSION 2.8)

project(auvir_model)

set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake )
message(STATUS "CMAKE_SOURCE_DIR=" ${CMAKE_CURRENT_SOURCE_DIR})
set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)

include_directories( SRC_DIR )

find_package(CUDA REQUIRED)
message(STATUS "cuda version:" ${CUDA_VERSION_STRING})
message(STATUS "cuda toolkit dir:" ${CUDA_TOOLKIT_ROOT_DIR})
message(STATUS "cuda sdk dir:" ${CUDA_SDK_ROOT_DIR})

find_package( OpenCV REQUIRED )
if( OPENCV_FOUND )
    message(STATUS "found opencv")
    add_executable (auvir_model main.cpp )
    add_executable(auvir_model ${SRC_DIR}/main.cpp)
    target_link_libraries(auvir_model ${OpenCV_LIBS})

else ( !OPENCV_FOUND )
    message(STATUS "not found opencv")
    message(STATUS ${CMAKE_ROOT})
    #link_directories("/usr/local/lib")
    #link_directories("/usr/local/cuda/lib64")
    link_directories("~/work/dev/thirdparty/opencv/build/lib/libopencv_imgproc.so
    ")

    #set ( THIRDPARTY_LIB_PATH_OPENCV "./../../thirdparty/build/opencv/" )
    set ( THIRDPARTY_LIB_PATH_OPENCV "/usr/local/share/OpenCV" )
    find_library( "libopencv_imgproc.so" ${THIRDPARTY_LIB_PATH_OPENCV} )
    find_library( "libopencv_highgui.so" ${THIRDPARTY_LIB_PATH_OPENCV} )
    find_library( "libopencv_core" ${THIRDPARTY_LIB_PATH_OPENCV} )
    add_executable(auvir_model ${SRC_DIR}/main.cpp )
    target_link_libraries(auvir_model "libopencv_core.so")
    target_link_libraries(auvir_model "libopencv_highgui.so")
    target_link_libraries(auvir_model "libopencv_imgproc.so")

endif( OPENCV_FOUND )

set ( Boost_USE_STATIC_LIBS OFF )
set ( Boost_USE_MULTITHREADED ON )
set ( Boost_USE_STATIC_RUNTIME OFF )

set (boost_components filesystem regex)

message (STATUS "boost_components:" ${boost_components})
message (STATUS "MAKE_MODULE_PATH:" ${CMAKE_MODULE_PATH})

#find_package ( BOOST REQUIRED)

