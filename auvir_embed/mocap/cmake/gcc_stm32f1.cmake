SET(CMAKE_C_FLAGS "-mthumb -fno-builtin -mcpu=cortex-m3 -Wall -std=c99 -ffunction-sections -fdata-sections -fomit-frame-pointer -mabi=aapcs -fno-unroll-loops -ffast-math -ftree-vectorize" CACHE INTERNAL "c compiler flags")
SET(CMAKE_CXX_FLAGS "-mthumb -fno-builtin -mcpu=cortex-m3 -Wall -std=c++11 -ffunction-sections -fdata-sections -fomit-frame-pointer -mabi=aapcs -fno-unroll-loops -ffast-math -ftree-vectorize" CACHE INTERNAL "cxx compiler flags")
SET(CMAKE_ASM_FLAGS "-mthumb -mcpu=cortex-m3 -x assembler-with-cpp" CACHE INTERNAL "asm compiler flags")

SET(CMAKE_EXE_LINKER_FLAGS "-Wl,--gc-sections -Wl,-Map=tut.map --specs=nosys.specs --specs=nano.specs -fno-exceptions -fno-rtti -mthumb -mcpu=cortex-m3 -mabi=aapcs" CACHE INTERNAL "executable linker flags")
#SET(CMAKE_EXE_LINKER_FLAGS "-Wl,--gc-sections -mthumb -mcpu=cortex-m3 -mabi=aapcs" CACHE INTERNAL "executable linker flags")
SET(CMAKE_MODULE_LINKER_FLAGS "-mthumb -mcpu=cortex-m3 -mabi=aapcs" CACHE INTERNAL "module linker flags")
SET(CMAKE_SHARED_LINKER_FLAGS "-mthumb -mcpu=cortex-m3 -mabi=aapcs" CACHE INTERNAL "shared linker flags")

SET(STM32_CHIP_TYPES 100xB 100xE 101x6 101xB 101xE 101xG 102x6 102xB 103x6 103xB 103xE 103xG 105xC 107xC CACHE INTERNAL "stm32f1 chip types")
SET(STM32_CODES "100.[468B]" "100.[CDE]" "101.[46]" "101.[8B]" "101.[CDE]" "101.[FG]" "102.[46]" "102.[8B]" "103.[46]" "103.[8B]" "103.[CDE]" "103.[FG]" "105.[8BC]" "107.[BC]")

MACRO(STM32_GET_CHIP_TYPE CHIP CHIP_TYPE)
    STRING(REGEX REPLACE "^[sS][tT][mM]32[fF](10[012357].[468BCDEFG]).+$" "\\1" STM32_CODE ${CHIP})
    SET(INDEX 0)
    FOREACH(C_TYPE ${STM32_CHIP_TYPES})
        LIST(GET STM32_CODES ${INDEX} CHIP_TYPE_REGEXP)
        IF(STM32_CODE MATCHES ${CHIP_TYPE_REGEXP})
            SET(RESULT_TYPE ${C_TYPE})
        ENDIF()
        MATH(EXPR INDEX "${INDEX}+1")
    ENDFOREACH()
    SET(${CHIP_TYPE} ${RESULT_TYPE})
ENDMACRO()

MACRO(STM32_GET_CHIP_PARAMETERS CHIP FLASH_SIZE RAM_SIZE)
    STRING(REGEX REPLACE "^[sS][tT][mM]32[fF](10[012357]).[468BCDEFG]" "\\1" STM32_CODE ${CHIP})
    STRING(REGEX REPLACE "^[sS][tT][mM]32[fF]10[012357].([468BCDEFG])" "\\1" STM32_SIZE_CODE ${CHIP})

    IF(STM32_SIZE_CODE STREQUAL "4")
        SET(FLASH "16K")
    ELSEIF(STM32_SIZE_CODE STREQUAL "6")
        SET(FLASH "32K")
    ELSEIF(STM32_SIZE_CODE STREQUAL "8")
        SET(FLASH "64K")
    ELSEIF(STM32_SIZE_CODE STREQUAL "B")
        SET(FLASH "128K")
    ELSEIF(STM32_SIZE_CODE STREQUAL "C")
        SET(FLASH "256K")
    ELSEIF(STM32_SIZE_CODE STREQUAL "D")
        SET(FLASH "384K")
    ELSEIF(STM32_SIZE_CODE STREQUAL "E")
        SET(FLASH "512K")
    ELSEIF(STM32_SIZE_CODE STREQUAL "F")
        SET(FLASH "768K")
    ELSEIF(STM32_SIZE_CODE STREQUAL "G")
        SET(FLASH "1024K")
    ENDIF()

    STM32_GET_CHIP_TYPE(${CHIP} TYPE)

    IF(${TYPE} STREQUAL 100xB)
        IF((STM32_SIZE_CODE STREQUAL "4") OR (STM32_SIZE_CODE STREQUAL "6"))
            SET(RAM "4K")
        ELSE()
            SET(RAM "8K")
        ENDIF()
    ELSEIF(${TYPE} STREQUAL 100xE)
        IF(STM32_SIZE_CODE STREQUAL "C")
            SET(RAM "24K")
        ELSE()
            SET(RAM "32K")
        ENDIF()
    ELSEIF(${TYPE} STREQUAL 101x6)
        IF(STM32_SIZE_CODE STREQUAL "4")
            SET(RAM "4K")
        ELSE()
            SET(RAM "6K")
        ENDIF()
    ELSEIF(${TYPE} STREQUAL 101xB)
        IF(STM32_SIZE_CODE STREQUAL "8")
            SET(RAM "10K")
        ELSE()
            SET(RAM "16K")
        ENDIF()
    ELSEIF(${TYPE} STREQUAL 101xE)
        IF(STM32_SIZE_CODE STREQUAL "C")
            SET(RAM "32K")
        ELSE()
            SET(RAM "48K")
        ENDIF()
    ELSEIF(${TYPE} STREQUAL 101xG)
        SET(RAM "80K")
    ELSEIF(${TYPE} STREQUAL 102x6)
        IF(STM32_SIZE_CODE STREQUAL "4")
            SET(RAM "4K")
        ELSE()
            SET(RAM "6K")
        ENDIF()
    ELSEIF(${TYPE} STREQUAL 102xB)
        IF(STM32_SIZE_CODE STREQUAL "8")
            SET(RAM "10K")
        ELSE()
            SET(RAM "16K")
        ENDIF()
    ELSEIF(${TYPE} STREQUAL 103x6)
        IF(STM32_SIZE_CODE STREQUAL "4")
            SET(RAM "6K")
        ELSE()
            SET(RAM "10K")
        ENDIF()
    ELSEIF(${TYPE} STREQUAL 103xB)
        SET(RAM "20K")
    ELSEIF(${TYPE} STREQUAL 103xE)
        IF(STM32_SIZE_CODE STREQUAL "C")
            SET(RAM "48K")
        ELSE()
            SET(RAM "54K")
        ENDIF()
    ELSEIF(${TYPE} STREQUAL 103xG)
        SET(RAM "96K")
    ELSEIF(${TYPE} STREQUAL 105xC)
        SET(RAM "64K")
    ELSEIF(${TYPE} STREQUAL 107xC)
        SET(RAM "64K")
    ENDIF()

    SET(${FLASH_SIZE} ${FLASH})
    SET(${RAM_SIZE} ${RAM})
ENDMACRO()

FUNCTION(STM32_SET_CHIP_DEFINITIONS TARGET CHIP_TYPE)
    LIST(FIND STM32_CHIP_TYPES ${CHIP_TYPE} TYPE_INDEX)
    IF(TYPE_INDEX EQUAL -1)
        MESSAGE(FATAL_ERROR "Invalid/unsupported STM32F1 chip type: ${CHIP_TYPE}")
    ENDIF()
    GET_TARGET_PROPERTY(TARGET_DEFS ${TARGET} COMPILE_DEFINITIONS)
    IF(TARGET_DEFS)
        SET(TARGET_DEFS "STM32F1;STM32F${CHIP_TYPE};${TARGET_DEFS}")
    ELSE()
        SET(TARGET_DEFS "STM32F1;STM32F${CHIP_TYPE}")
    ENDIF()
    SET_TARGET_PROPERTIES(${TARGET} PROPERTIES COMPILE_DEFINITIONS "${TARGET_DEFS}")
ENDFUNCTION()
