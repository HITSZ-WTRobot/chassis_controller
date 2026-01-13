# UserCode/CMakeLists.txt
cmake_minimum_required(VERSION 3.21)

set(LIB_NAME chassis_controller)

project(${LIB_NAME})

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

# ---------------------------------------------------------------------------
# feature options: option + source + macro
# ---------------------------------------------------------------------------
set(DRIVER_MACROS "")

set(ChassisIF_ChassisType "" CACHE STRING "select chassis type (required)")
set_property(CACHE ChassisIF_ChassisType PROPERTY STRINGS
        MECANUM4
        OMNI4
)

if (ChassisIF_ChassisType STREQUAL "MECANUM4")
    list(APPEND ALL_SOURCES drivers/chassis_mecanum4.c)
    list(APPEND ALL_HEADERS "drivers/chassis_mecanum4.h")
    list(APPEND DRIVER_MACROS CHASSIS_MECANUM4)
elseif (ChassisIF_ChassisType STREQUAL "OMNI4")
    list(APPEND ALL_SOURCES drivers/chassis_omni4.c)
    list(APPEND ALL_HEADERS "drivers/chassis_omni4.h")
    list(APPEND DRIVER_MACROS CHASSIS_OMNI)
else ()
    message(FATAL_ERROR "invalid ChassisIF_ChassisType: ${ChassisIF_ChassisType}")
endif ()

# ---------------------------------------------------------------------------
# create static library
# ---------------------------------------------------------------------------
add_library(chassis_controller STATIC ${ALL_SOURCES})
if (NOT TARGET ${LIB_NAME})
    message(FATAL_ERROR "target ${LIB_NAME} not created")
endif ()

# ---------------------------------------------------------------------------
# compile definitions
# ---------------------------------------------------------------------------
if (DRIVER_MACROS)
    target_compile_definitions(${LIB_NAME} PUBLIC ${DRIVER_MACROS})
endif ()

# ---------------------------------------------------------------------------
# build/include 目录
# ---------------------------------------------------------------------------
set(EXPORT_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/include)
file(MAKE_DIRECTORY ${EXPORT_INCLUDE_DIR})
target_include_directories(${LIB_NAME} PUBLIC ${EXPORT_INCLUDE_DIR})

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
target_link_libraries(${LIB_NAME} PUBLIC stm32cubemx)
add_dependencies(${LIB_NAME} stm32cubemx)

# Repo: https://github.com/HITSZ-WTRobot/motor_drivers
target_link_libraries(${LIB_NAME} PUBLIC motor_drivers)
add_dependencies(${LIB_NAME} motor_drivers)

# Repo: https://github.com/HITSZ-WTRobot/s-curve-planner
target_link_libraries(${LIB_NAME} PUBLIC s_curve)
add_dependencies(${LIB_NAME} s_curve)

# Repo: https://github.com/HITSZ-WTRobot/C_Library.git
target_link_libraries(${LIB_NAME} PUBLIC libs_mit_pd)
add_dependencies(${LIB_NAME} libs_mit_pd)

target_link_libraries(${LIB_NAME} PUBLIC m)
