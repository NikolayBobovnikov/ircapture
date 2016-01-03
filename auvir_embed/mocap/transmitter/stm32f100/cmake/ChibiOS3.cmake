IF(NOT ChibiOS_FIND_COMPONENTS) 
    SET(ChibiOS_FIND_COMPONENTS nil hal st)
    MESSAGE(STATUS "No ChibiOS components specified, using default: ${ChibiOS_FIND_COMPONENTS}")
ENDIF()

LIST(FIND ChibiOS_FIND_COMPONENTS nil ChibiOS_FIND_COMPONENTS_nil)
LIST(FIND ChibiOS_FIND_COMPONENTS rt ChibiOS_FIND_COMPONENTS_rt)
LIST(FIND ChibiOS_FIND_COMPONENTS hal ChibiOS_FIND_COMPONENTS_hal)
LIST(FIND ChibiOS_FIND_COMPONENTS st ChibiOS_FIND_COMPONENTS_st)

IF((${ChibiOS_FIND_COMPONENTS_nil} LESS 0) AND (${ChibiOS_FIND_COMPONENTS_rt} LESS 0))
  MESSAGE(STATUS "No kernel component selected, using Nil kernel")
  LIST(APPEND ChibiOS_FIND_COMPONENTS nil)
  SET(CHIBIOS_KERNEL nil)
ELSE()
  IF((NOT (${ChibiOS_FIND_COMPONENTS_nil} LESS 0)) AND (NOT (${ChibiOS_FIND_COMPONENTS_rt} LESS 0)))
    MESSAGE(FATAL_ERROR "Cannot use RT and Nil kernel at the same time")
  ENDIF()
  IF(NOT (${ChibiOS_FIND_COMPONENTS_nil} LESS 0))
    SET(CHIBIOS_KERNEL nil)
  ELSE()
    SET(CHIBIOS_KERNEL rt)
  ENDIF()
ENDIF()

IF(${ChibiOS_FIND_COMPONENTS_hal} LESS 0)
  LIST(APPEND ChibiOS_FIND_COMPONENTS hal)
ENDIF()

IF(${ChibiOS_FIND_COMPONENTS_st} LESS 0)
  LIST(APPEND ChibiOS_FIND_COMPONENTS st)
ENDIF()
  
INCLUDE(ChibiOS3_LD)
INCLUDE(ChibiOS3_HAL)

IF(${CHIBIOS_KERNEL} STREQUAL rt)
  INCLUDE(ChibiOS3_RT)
ELSE()
  INCLUDE(ChibiOS3_NIL)
ENDIF()

SET(CHIBIOS_COMPONENTS nil rt hal ${CHIBIOS_HAL_MODULES})

IF(NOT ChibiOS_LINKER_SCRIPT)
    MESSAGE(STATUS "ChibiOS doesn't have linker script for your chip, please specify it directly using ChibiOS_LINKER_SCRIPT variable.")
ENDIF()

FOREACH(comp ${ChibiOS_FIND_COMPONENTS}) 
    LIST(FIND CHIBIOS_COMPONENTS ${comp} INDEX)
    IF(INDEX EQUAL -1)
        MESSAGE(FATAL_ERROR "Unknown ChibiOS component: ${comp}\nSupported ChibiOS components: ${CHIBIOS_COMPONENTS}")
    ENDIF()
    FOREACH(source ${CHIBIOS_${comp}_SOURCES})
        FIND_FILE(CHIBIOS_${comp}_${source} NAMES ${source} PATHS ${CHIBIOS_${comp}_SEARCH_PATH} NO_DEFAULT_PATH CMAKE_FIND_ROOT_PATH_BOTH)
        LIST(APPEND ChibiOS_SOURCES ${CHIBIOS_${comp}_${source}})
    ENDFOREACH()
    IF(CHIBIOS_${comp}_SEARCH_HEADERS)
        FOREACH(header ${CHIBIOS_${comp}_SEARCH_HEADERS})
            FIND_PATH(CHIBIOS_${comp}_${header}_INCLUDE_DIR NAMES ${header} PATHS ${CHIBIOS_${comp}_SEARCH_PATH} NO_DEFAULT_PATH CMAKE_FIND_ROOT_PATH_BOTH)
            LIST(APPEND ChibiOS_INCLUDE_DIRS ${CHIBIOS_${comp}_${header}_INCLUDE_DIR})
        ENDFOREACH()
    ENDIF()
    IF(CHIBIOS_${comp}_PLATFORM_SEARCH_HEADERS)
        FOREACH(header ${CHIBIOS_${comp}_PLATFORM_SEARCH_HEADERS})
            FIND_PATH(CHIBIOS_${comp}_PLATFORM_${header}_INCLUDE_DIR NAMES ${header} PATHS ${CHIBIOS_${comp}_PLATFORM_SEARCH_PATH} NO_DEFAULT_PATH CMAKE_FIND_ROOT_PATH_BOTH)
            LIST(APPEND ChibiOS_INCLUDE_DIRS ${CHIBIOS_${comp}_PLATFORM_${header}_INCLUDE_DIR})
        ENDFOREACH()
    ENDIF()
    IF(CHIBIOS_${comp}_PLATFORM_SOURCES)
        FOREACH(source ${CHIBIOS_${comp}_PLATFORM_SOURCES})
            FIND_FILE(CHIBIOS_${comp}_PLATFORM_${source} NAMES ${source} PATHS ${CHIBIOS_${comp}_PLATFORM_SEARCH_PATH} NO_DEFAULT_PATH CMAKE_FIND_ROOT_PATH_BOTH)
            LIST(APPEND ChibiOS_SOURCES ${CHIBIOS_${comp}_PLATFORM_${source}})
        ENDFOREACH()
    ENDIF()
ENDFOREACH()