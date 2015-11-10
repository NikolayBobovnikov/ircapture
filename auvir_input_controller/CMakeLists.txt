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

# boost
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
find_package(Boost COMPONENTS system regex filesystem REQUIRED)

message(STATUS "Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
include_directories(${Boost_INCLUDE_DIRS})

# include source/include files recursively
file(GLOB_RECURSE PROJECT_SOURCES "${SRC_DIR}/*.cpp")
file(GLOB_RECURSE PROJECT_HEADERS "${SRC_DIR}/*.h" "${SRC_DIR}/*.hpp")

set (INCLUDE_DIRS "")
foreach (_headerFile ${PROJECT_HEADERS})
    get_filename_component(_dir ${_headerFile} PATH)
    list (APPEND INCLUDE_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES INCLUDE_DIRS)

include_directories(${INCLUDE_DIRS})
add_executable (${CMAKE_PROJECT_NAME}
                ${PROJECT_SOURCES}
                )

target_link_libraries(  ${CMAKE_PROJECT_NAME}
                        ${Boost_LIBRARIES}
                     )


