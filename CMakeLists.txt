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
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(adxl382_spi C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
if (TARGET tinyusb_device) # check if tinyusb is available
    add_executable(adxl382_spi
            adxl382.c
            main.c
            )
    #enable uart and usb port
    pico_enable_stdio_uart(adxl382_spi 0)
    pico_enable_stdio_usb(adxl382_spi 1)
    # pull in common dependencies and additional spi hardware support
    target_link_libraries(adxl382_spi pico_stdlib hardware_spi)

    # create map/bin/hex file etc.
    pico_add_extra_outputs(adxl382_spi)

# add url via pico_set_program_url
elseif(PICO_ON_DEVICE)
    message("Skipping adxl382_spi because TinyUSB submodule is not initialized in the SDK")
endif()