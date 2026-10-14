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


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Project1)

include(cmake/gcc-arm-none-eabi.cmake)
# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

option(SYSTEM_IDENTIFICATION_MODE "Build the independent yaw identification task" OFF)
option(VELOCITY_CONTROL_TEST_MODE "Build the yaw velocity PID/feedforward comparison task" OFF)
option(GIMBAL_LEGACY_ASCII_COMMANDS "Enable legacy WASD gimbal commands" OFF)

if(SYSTEM_IDENTIFICATION_MODE AND VELOCITY_CONTROL_TEST_MODE)
    message(FATAL_ERROR "SYSTEM_IDENTIFICATION_MODE and VELOCITY_CONTROL_TEST_MODE are mutually exclusive")
endif()

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        BSP/bsp_usb_cdc.c
        BSP/BMI088Middleware.c
        BSP/BMI088driver.c
        Algorithm/MahonyAHRS.c
        Algorithm/pid.c
        BSP/bsp_can.c
        Modules/can_motor.c
        Tasks/usb_task.c
        Tasks/imu_task.c
        Tasks/gimbal_protocol.c
        Tasks/gimbal_task.c
        Tasks/system_identification_task.c
        Tasks/velocity_control_test_task.c)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        BSP
        Algorithm
        Modules
        Tasks
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    $<$<BOOL:${SYSTEM_IDENTIFICATION_MODE}>:SYSTEM_IDENTIFICATION_MODE=1>
    $<$<NOT:$<BOOL:${SYSTEM_IDENTIFICATION_MODE}>>:SYSTEM_IDENTIFICATION_MODE=0>
    $<$<BOOL:${VELOCITY_CONTROL_TEST_MODE}>:VELOCITY_CONTROL_TEST_MODE=1>
    $<$<NOT:$<BOOL:${VELOCITY_CONTROL_TEST_MODE}>>:VELOCITY_CONTROL_TEST_MODE=0>
    $<$<BOOL:${GIMBAL_LEGACY_ASCII_COMMANDS}>:GIMBAL_LEGACY_ASCII_COMMANDS=1>
    $<$<NOT:$<BOOL:${GIMBAL_LEGACY_ASCII_COMMANDS}>>:GIMBAL_LEGACY_ASCII_COMMANDS=0>
)



# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Enable floating-point formatting for VOFA+ FireWater ASCII output.
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)
