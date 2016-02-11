#set(CMAKE_CXX_COMPILER /usr/bin/clang++)
cmake_minimum_required(VERSION 2.8)
project(auvir_input_controller)
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER}")
#save build information
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# TODO: set to RELEASE in production
set(CMAKE_BUILD_TYPE DEBUG)
# set cmake modules path
set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake )
add_definitions(-std=c++11)
#add_definitions(-DBOOST_ALL_DYN_LINK)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# C++14: set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y")

# set project source directory
set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)

#============================ boost
#set (Boost_DEBUG ON)
#set (Boost_NO_SYSTEM_PATHS ON)
set (BOOST_ROOT "..\\..\\thirdparty\\boost\\boost_1_59_0" )
#set (BOOST_INCLUDEDIR "..\\..\\thirdparty\\boost\\boost_1_59_0\\")
set (BOOST_LIBRARYDIR "..\\..\\thirdparty\\boost\\boost_1_59_0\\stage\\lib")
#set (BOOST_LIBRARYDIR "..\\..\\thirdparty\\boost\\boost_1_59_0\\lib")

add_definitions( -DBOOST_ALL_NO_LIB )
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost COMPONENTS system regex filesystem thread REQUIRED)

message(STATUS "Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
include_directories(${Boost_INCLUDE_DIRS})

#============================ libusb

#============================ hidapi
set(HIDAPI_DIR ${CMAKE_SOURCE_DIR}/libraries/hidapi)

if(UNIX)
    message(STATUS "using linux system!")
    set(HIDAPI_OS_SPECIFIC_DIR ${HIDAPI_DIR}/linux)
endif(UNIX)
if(WIN32)
    message(STATUS "using windows system!")
    set(HIDAPI_OS_SPECIFIC_DIR ${HIDAPI_DIR}/windows)
endif(WIN32)

SET(LIB_DIRS
    ${SRC_DIR}
    ${HIDAPI_DIR}/hidapi ${HIDAPI_OS_SPECIFIC_DIR} ${HIDAPI_DIR}/hidtest
)

####### Recursive addition of library source and header files from all used libraries #######
SET(ALL_LIB_SOURCES "")
SET(ALL_LIB_HEADERS "")

# include source/include files recursively
foreach (_dir ${LIB_DIRS})
    file(GLOB_RECURSE LIB_SOURCES "${_dir}/*.cpp" "${_dir}/*.c")
    file(GLOB_RECURSE LIB_HEADERS "${_dir}/*.hpp" "${_dir}/*.h")
    list(APPEND ALL_LIB_SOURCES ${LIB_SOURCES})
    list(APPEND ALL_LIB_HEADERS ${LIB_HEADERS})
endforeach()

list(REMOVE_DUPLICATES ALL_LIB_HEADERS)
list(REMOVE_DUPLICATES ALL_LIB_SOURCES)


# include directory for every header file
SET (INCLUDE_LIB_DIRS "")
foreach (_headerFile ${ALL_LIB_HEADERS})
    get_filename_component(_dir ${_headerFile} PATH)
    list (APPEND INCLUDE_LIB_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES INCLUDE_LIB_DIRS)

#############################################################################################

include_directories(
${INCLUDE_LIB_DIRS}
)

# Build everything into elf
ADD_EXECUTABLE(
    ${CMAKE_PROJECT_NAME}
    ${ALL_LIB_SOURCES}
    ${ALL_LIB_HEADERS}
    )

target_link_libraries(
    ${CMAKE_PROJECT_NAME}
    ${Boost_LIBRARIES}
    /usr/lib/x86_64-linux-gnu/libudev.so
)


