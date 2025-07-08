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
set(CMAKE_PROJECT_NAME cboard)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    algorithms/math_lib.cpp
    algorithms/VMC_calc.cpp
    algorithms/kalman_filter.cpp

    applications/sources/led_task.cpp
    applications/sources/buzzer_task.cpp
    applications/sources/uart_task.cpp
    applications/sources/can_task.cpp
    applications/sources/plot_task.cpp
    applications/sources/imu_task.cpp
    applications/sources/calibrate_task.cpp
    applications/sources/observe_task.cpp
    applications/sources/chassis_L_task.cpp
    applications/sources/chassis_R_task.cpp


    sp_middleware/io/led/led.cpp
    sp_middleware/io/buzzer/buzzer.cpp 
    sp_middleware/io/dbus/dbus.cpp
    sp_middleware/io/can/can.cpp
    sp_middleware/io/plotter/plotter.cpp
    sp_middleware/io/bmi088/bmi088.cpp
    
    sp_middleware/motor/dm_motor/dm_motor.cpp

    sp_middleware/tools/pid/pid.cpp
    sp_middleware/tools/math_tools/math_tools.cpp
    sp_middleware/tools/mahony/mahony.cpp
    sp_middleware/tools/low_pass_filter/low_pass_filter.cpp

)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    sp_middleware
    applications/includings
    algorithms
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
