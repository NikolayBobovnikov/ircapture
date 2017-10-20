cmake_minimum_required(VERSION 3.0)

################################################
#TODO: add/fix functions to include only those files which has to be included (those defined in this file, like SPL modules)
macro(IS_THERE value list )
  foreach (_item ${list})
    message(STATUS "_item: " ${_item})
    message(STATUS "value: " ${value})
    if((${_item} MATCHES ${value}) OR (${_item} EQUAL ${value}))
      message(STATUS "SET result to 1 " )
      SET(${result_} 1)
    endif()
  endforeach()
endmacro(IS_THERE)


function(IS_REQUIRED_FUNC _file _required_src_list _is_required)
  get_filename_component(_filename ${_file} NAME)
  foreach (_required_src ${_required_src_list})
    if((${_file} MATCHES ${_required_src}) OR (${_filename} EQUAL ${_required_src}))
      SET(${_is_required} 1 PARENT_SCOPE)
      message(STATUS "in function: add ${_filename}")
    endif()
  endforeach()
endfunction(IS_REQUIRED_FUNC)

function(IS_INCLUDED_MODULE_FUNC _file _module_list _is_included)
  get_filename_component(_filename ${_file} NAME)
  foreach (_module ${_module_list})
    message(STATUS "check module:  ${_module}")
    if((${_file} MATCHES ${_module}) OR (${_filename} IN_LIST _module_list))
      SET(${_is_included} 1 PARENT_SCOPE)
      message(STATUS "in function: add ${_filename}")
    endif()
  endforeach()
endfunction(IS_INCLUDED_MODULE_FUNC)

####### Recursive addition of library source and header files from all used libraries #######
SET(ALL_PROJECT_SOURCES "")
SET(ALL_PROJECT_HEADERS "")
SET(INCLUDE_SRC_DIRS "")
# include source/include files recursively
foreach (_dir ${SRC_DIRS})
  file(GLOB_RECURSE LIB_SOURCES "${_dir}/*.c" "${_dir}/*.cpp" "${_dir}/*.s")
  file(GLOB_RECURSE LIB_HEADERS "${_dir}/*.h")

  # filter out temporary files etc, like .#main.cpp
  foreach (_file ${LIB_SOURCES})
    SET(_file_filtered "")
    get_filename_component(_filedir ${_file} PATH)
    #filter source code file names by regex - c++,c,asm files
    string(REGEX MATCHALL "${_filedir}/[A-Za-z0-9_]+.(cpp|c|s)" _file_filtered ${_file})
    list(APPEND ALL_PROJECT_SOURCES ${_file_filtered})
  endforeach()

  foreach (_file ${LIB_HEADERS})
    SET(_file_filtered "")
    get_filename_component(_filedir ${_file} PATH)
    string(REGEX MATCHALL "${_filedir}/[A-Za-z0-9_]+.(hpp|h)" _file_filtered ${_file})
    list(APPEND ALL_PROJECT_HEADERS ${_file_filtered})
    get_filename_component(_dir ${_file} PATH)
    list (APPEND INCLUDE_SRC_DIRS ${_dir})
  endforeach()

endforeach()

list(REMOVE_DUPLICATES ALL_PROJECT_HEADERS)
list(REMOVE_DUPLICATES ALL_PROJECT_SOURCES)
list(REMOVE_DUPLICATES INCLUDE_SRC_DIRS)

message(STATUS "Source files to be compiled: ")
SET(ALL_PROJECT_SOURCE_NAMES "")
foreach (_src_path ${ALL_PROJECT_SOURCES})
  get_filename_component(_curr_file_name ${_src_path} NAME)
  list(APPEND ALL_PROJECT_SOURCE_NAMES ${_curr_file_name})
  #message(STATUS "--> ${_curr_file_name}")
endforeach()

message(STATUS "Header files: ")
SET(ALL_PROJECT_HEADER_NAMES "")
foreach (_src_path ${ALL_PROJECT_HEADERS})
  get_filename_component(_curr_file_name ${_src_path} NAME)
  list(APPEND ALL_PROJECT_HEADER_NAMES ${_curr_file_name})
  #message(STATUS "--> ${_curr_file_name}")
endforeach()

# for cmake-ide in emacs: save project dir/build dir to dir-locals.el
message(STATUS "Create .dir-locals.el")
file(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/.dir-locals.el
  "((nil . (
  (cmake-ide-project-dir . \"${CMAKE_CURRENT_SOURCE_DIR}/\")
  (cmake-ide-build-dir . \"${PROJECT_BINARY_DIR}/\")
 ))
 ((c-mode . ((mode . c++))))
)")


include_directories(
  ${INCLUDE_SRC_DIRS}
  )

#supress warnings from library code
include_directories(
  SYSTEM ${SRC_DIRS_SYSTEM}
  )

MESSAGE(STATUS "PROJECT_BINARY_DIR : " ${PROJECT_BINARY_DIR})
# Build everything into elf
ADD_EXECUTABLE(
  ${PROJECT_NAME}
  ${ALL_PROJECT_SOURCES}
  ${ALL_PROJECT_HEADERS}
  )

STM32_SET_TARGET_PROPERTIES(${PROJECT_NAME})
#STM32_ADD_HEX_BIN_TARGETS(${PROJECT_NAME})

# Convert 'elf' into 'hex' and 'bin'
ADD_CUSTOM_COMMAND(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_OBJCOPY} ARGS -Oihex ${PROJECT_NAME} ${PROJECT_NAME}.hex)
ADD_CUSTOM_COMMAND(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_OBJCOPY} ARGS -Obinary ${PROJECT_NAME} ${PROJECT_NAME}.bin)
