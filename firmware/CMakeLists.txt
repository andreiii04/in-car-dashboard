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
set(CMAKE_PROJECT_NAME dashboard_firmware)

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
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    App/Src/i2c_bus.c
    App/Src/lsm6dso_platform.c
    App/Src/bme68x_platform.c
    App/Src/imu.c
    App/Src/env.c
    App/Src/uart_bus.c
    App/Src/nmea.c
    App/Src/gps.c
    App/Src/attitude.c
    App/Src/trip.c
    App/Src/elevation.c
    App/Src/usb_serial.c
    App/Src/lcd.c
    App/Src/gfx.c
    App/Src/screen.c
    App/Src/sd_crc.c
    App/Src/sd_spi.c
    App/Src/session_record.c
    App/Src/session_log.c
    App/Src/console.c
    App/Src/bench_cal.c
    App/Fonts/font_small.c
    App/Fonts/font_medium.c
    App/Fonts/font_large.c
    App/Drivers/lsm6dso/lsm6dso_reg.c
    App/Drivers/bme68x/bme68x.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    App/Inc
    App/Drivers/lsm6dso
    App/Drivers/bme68x
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# newlib-nano leaves float formatting out of printf unless it is pulled in
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)

list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
