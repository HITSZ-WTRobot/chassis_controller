# UserCode/CMakeLists.txt
cmake_minimum_required(VERSION 3.21)
project(chassis_controller)

# 导库方式
# add_subdirectory({project_path}/UserCode)
# target_link_libraries(${PROJECT_NAME}.elf PRIVATE chassis_controller)

# ---------------------------------------------------------------------------
# guard: UserCode should only be included by parent project
# ---------------------------------------------------------------------------
if (PROJECT_IS_TOP_LEVEL)
    message(FATAL_ERROR "UserCode must be included via add_subdirectory() from parent project")
endif ()

# ---------------------------------------------------------------------------
# layer options
# ---------------------------------------------------------------------------
#option(ENABLE_BSP "enable BSP layer" ON)

# ---------------------------------------------------------------------------
# collect layer sources
# ---------------------------------------------------------------------------
set(ALL_SOURCES interfaces/chassis_if.c)

set(ALL_HEADERS "interfaces/chassis_if.h")

#if (ENABLE_BSP)
#    file(GLOB_RECURSE BSP_SOURCES bsp/*.c)
#    list(APPEND ALL_SOURCES ${BSP_SOURCES})
#    list(APPEND ALL_HEADERS
#            "bsp/can_driver.h"
#            "bsp/gpio_driver.h"
#            "bsp/pwm.h"
#    )
#endif ()

# ---------------------------------------------------------------------------
# feature options: option + source + macro
# ---------------------------------------------------------------------------
set(DRIVER_MACROS "")

set(CHASSIS_TYPE "" CACHE STRING "select chassis type (required)")
set_property(CACHE CHASSIS_TYPE PROPERTY STRINGS
        MECANUM4
        OMNI4
)

if (CHASSIS_TYPE STREQUAL "MECANUM4")
    list(APPEND ALL_SOURCES drivers/chassis_mecanum4.c)
    list(APPEND ALL_HEADERS "drivers/chassis_mecanum4.h")
    list(APPEND DRIVER_MACROS USE_MECANUM4)
elseif (CHASSIS_TYPE STREQUAL "OMNI4")
    list(APPEND ALL_SOURCES drivers/chassis_omni4.c)
    list(APPEND ALL_HEADERS "drivers/chassis_omni4.h")
    list(APPEND DRIVER_MACROS USE_OMNI)
else ()
    message(FATAL_ERROR "invalid CHASSIS_TYPE: ${CHASSIS_TYPE}")
endif ()

# ---------------------------------------------------------------------------
# create static library
# ---------------------------------------------------------------------------
add_library(${PROJECT_NAME} STATIC ${ALL_SOURCES})

# ---------------------------------------------------------------------------
# compile definitions
# ---------------------------------------------------------------------------
if (DRIVER_MACROS)
    target_compile_definitions(${PROJECT_NAME} PUBLIC ${DRIVER_MACROS})
endif ()

# ---------------------------------------------------------------------------
# build/include 目录
# ---------------------------------------------------------------------------
set(EXPORT_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/include)
file(MAKE_DIRECTORY ${EXPORT_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} PUBLIC ${EXPORT_INCLUDE_DIR})

# ---------------------------------------------------------------------------
# copy selected headers
# ---------------------------------------------------------------------------
foreach (header IN LISTS ALL_HEADERS)
    configure_file(${CMAKE_CURRENT_LIST_DIR}/${header}
            ${EXPORT_INCLUDE_DIR}/${header} COPYONLY)
endforeach ()


# ---------------------------------------------------------------------------
# declare dependencies
# ---------------------------------------------------------------------------
add_dependencies(${PROJECT_NAME} stm32cubemx)

# Repo: https://github.com/HITSZ-WTRobot/motor_drivers
add_dependencies(${PROJECT_NAME} motor_drivers)