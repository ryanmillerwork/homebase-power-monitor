# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD waveshare_rp2040_zero CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(power_monitor C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(power_monitor power_monitor.c)

pico_set_program_name(power_monitor "power_monitor")

# Firmware version string used both for binary metadata and the JSON API.
# You can override at build time, e.g.:
#   cmake .. -DFW_VERSION=1.2.3
set(FW_VERSION "0.1" CACHE STRING "Firmware version string")

# If FW_VERSION hasn't been overridden, try to derive it from git (nice for dev builds).
if (FW_VERSION STREQUAL "0.1")
    find_package(Git QUIET)
    if (GIT_FOUND AND EXISTS "${CMAKE_CURRENT_LIST_DIR}/.git")
        execute_process(
            COMMAND ${GIT_EXECUTABLE} describe --tags --dirty --always
            WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
            OUTPUT_VARIABLE GIT_DESCRIBE
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
        )
        if (GIT_DESCRIBE)
            set(FW_VERSION "${GIT_DESCRIBE}" CACHE STRING "Firmware version string" FORCE)
        endif()
    endif()
endif()

pico_set_program_version(power_monitor "${FW_VERSION}")

# Enable only stdio over USB (CDC)
pico_enable_stdio_uart(power_monitor 0)
pico_enable_stdio_usb(power_monitor 1)

# Add the standard library to the build
target_link_libraries(power_monitor
        pico_stdlib
        hardware_i2c
        hardware_flash)

# Ensure TinyUSB uses our custom strings for the device descriptor
target_compile_definitions(power_monitor PRIVATE
        USBD_MANUFACTURER="Homebase"
        USBD_PRODUCT="power_monitor"
        FW_VERSION="${FW_VERSION}"
)

# Add the standard include files to the build
target_include_directories(power_monitor PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(power_monitor)

