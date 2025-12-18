cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

#Uncomment for hardware floating point
add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)

add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)


# Define the build type
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif ()

# Set the project name
set(CMAKE_PROJECT_NAME chassis_controller)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined library search paths
)

# Add sources to executable
file(GLOB_RECURSE SOURCES "UserCode/*.*")
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user sources here
        ${SOURCES}
)

# Add include paths
include_directories(UserCode)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined symbols
        CHASSIS_MECANUM4
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

set(USER_LIBRARIES "")
# Add dependence from library
# ===================== DEPENDENCIES =====================

# e.g.
#add_subdirectory(library/motor_drivers/UserCode)

# motor_drivers
# Repo: https://github.com/HITSZ-WTRobot/motor_drivers
set(ENABLE_BSP ON) # 启用 bsp
set(ENABLE_LIBS ON) # 启用 lib
set(ENABLE_CONTROLLERS ON) # 启用 controller

set(ENABLE_DJI_DRIVER ON)
set(ENABLE_TB6612_DRIVER OFF)
set(ENABLE_VESC_DRIVER OFF)
set(ENABLE_DM_DRIVER OFF)
add_subdirectory(Modules/motor_drivers/UserCode)
list(APPEND USER_LIBRARIES motor_drivers)

add_subdirectory(Modules/HWT101CT_driver/UserCode)
list(APPEND USER_LIBRARIES HWT101CT)

# =======================================================

# every library will depend on stm32cubemx
foreach (LIBRARY IN LISTS USER_LIBRARIES)
    target_link_libraries(${LIBRARY} PRIVATE stm32cubemx)
endforeach ()

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
        stm32cubemx

        # Add user defined libraries
        ${USER_LIBRARIES}
)
