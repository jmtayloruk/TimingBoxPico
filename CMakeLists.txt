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
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(picoTimingBoxFirmware C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(picoTimingBoxFirmware picoTimingBoxFirmware.c )

pico_set_program_name(picoTimingBoxFirmware "picoTimingBoxFirmware")
pico_set_program_version(picoTimingBoxFirmware "1.1")

# Generate PIO header
pico_generate_pio_header(picoTimingBoxFirmware ${CMAKE_CURRENT_LIST_DIR}/pianolaPinDriver.pio)
pico_generate_pio_header(picoTimingBoxFirmware ${CMAKE_CURRENT_LIST_DIR}/pianola24bitClock.pio)
# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(picoTimingBoxFirmware 0)
# Optional development build: native USB exposes the SDK picotool reset interface.
# Keep production USB-off timing unchanged unless explicitly requested at configure.
option(TIMINGBOX_USB_FLASHING "Enable native USB for software-assisted flashing" OFF)
if(TIMINGBOX_USB_FLASHING)
    pico_enable_stdio_usb(picoTimingBoxFirmware 1)
    target_compile_definitions(picoTimingBoxFirmware PRIVATE
        PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=0
        PICO_STDIO_USB_STDOUT_TIMEOUT_US=0
        PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE=1
        PICO_STDIO_USB_ENABLE_RESET_VIA_BAUD_RATE=1)
else()
    pico_enable_stdio_usb(picoTimingBoxFirmware 0)
endif()

# Add the standard library to the build
target_link_libraries(picoTimingBoxFirmware
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(picoTimingBoxFirmware PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(picoTimingBoxFirmware 
        hardware_pio
        pico_multicore
        )

pico_add_extra_outputs(picoTimingBoxFirmware)

