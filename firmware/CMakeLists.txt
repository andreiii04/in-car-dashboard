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
    App/sensors/i2c_bus.c
    App/sensors/lsm6dso_platform.c
    App/sensors/bme68x_platform.c
    App/sensors/imu.c
    App/sensors/env.c
    App/sensors/air.c
    App/gps/uart_bus.c
    App/gps/nmea.c
    App/gps/gps.c
    App/fusion/attitude.c
    App/fusion/trip.c
    App/fusion/elevation.c
    App/console/usb_serial.c
    App/display/lcd.c
    App/display/gfx.c
    App/display/screen.c
    App/storage/sd_crc.c
    App/storage/sd_spi.c
    App/storage/session_record.c
    App/storage/session_log.c
    App/storage/air_slot.c
    App/storage/air_store.c
    App/console/console.c
    App/sensors/bench_cal.c
    App/rtos/app_state.c
    App/rtos/app_tasks.c
    App/rtos/watchdog.c
    App/fonts/font_small.c
    App/fonts/font_medium.c
    App/fonts/font_large.c
    App/drivers/lsm6dso/lsm6dso_reg.c
    App/drivers/bme68x/bme68x.c
    # newlib's malloc and stdio locks for FreeRTOS; CubeMX generates this into
    # the firmware root (with stm32_lock.h next to it) but leaves it out of its
    # own cmake file, and without it the locks are newlib's empty stubs
    newlib_lock_glue.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    App/sensors
    App/gps
    App/fusion
    App/display
    App/storage
    App/console
    App/rtos
    App/drivers/lsm6dso
    App/drivers/bme68x
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Bosch BSEC2 for the air quality numbers - closed source, not in the repo
# (the README says where to get it). linked only if it is there; without it
# air.c builds as a stand-in and the BME680 gives raw readings, as before
set(BSEC_DIR ${CMAKE_SOURCE_DIR}/App/drivers/bsec/algo/bsec_IAQ)
set(BSEC_CONFIG_DIR ${BSEC_DIR}/config/bme680/bme680_iaq_33v_3s_4d)
if(EXISTS ${BSEC_DIR}/bin/gcc/Cortex_M4F/libalgobsec.a)
    message(STATUS "BSEC found - air quality on")
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${BSEC_CONFIG_DIR}/bsec_iaq.c)
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${BSEC_DIR}/inc ${BSEC_CONFIG_DIR})
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE HAVE_BSEC=1)
    # m again after it - the library calls expf, log10f and friends, and the
    # linker only looks backwards for them
    target_link_libraries(${CMAKE_PROJECT_NAME} ${BSEC_DIR}/bin/gcc/Cortex_M4F/libalgobsec.a m)
else()
    message(STATUS "BSEC not found - raw BME680 readings only")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE HAVE_BSEC=0)
endif()

# newlib-nano leaves float formatting out of printf unless it is pulled in
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)

list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
