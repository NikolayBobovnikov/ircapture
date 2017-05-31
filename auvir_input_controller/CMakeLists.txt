include("cmake/HunterGate.cmake")
# install hunter package manager
HunterGate(
    URL "https://github.com/ruslo/hunter/archive/v0.15.32.tar.gz"
    SHA1 "0bc311b87b4c1c0ab51f091e3bffb47258afb708"
)

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
add_definitions(-std=c++14)
# C++14: set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y")

# set project source directory
set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)


if(WIN32)
    add_definitions(-D_WIN32_WINNT=_WIN32_WINNT_WIN7)
endif()

#============================ boost
set(Boost_USE_STATIC_LIBS ON)
if(MSVC)
  add_definitions(-DBOOST_ALL_NO_LIB=1)
endif()

# download boost
hunter_add_package(Boost COMPONENTS system filesystem regex thread)
# now boost can be used
find_package(Boost CONFIG REQUIRED system filesystem regex thread)

message(STATUS "Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
include_directories(${Boost_INCLUDE_DIRS})

#============================ libusb


SET(LIB_DIRS
    ${SRC_DIR}
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
)


