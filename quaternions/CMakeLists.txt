# Generated CMake Pico project file

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
set(sdkVersion 2.0.0)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

# Define project and target name
project(mputest C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable with the main source file `mputest.c`
add_executable(mputest mputest.c)

# Set program name and version
pico_set_program_name(mputest "mputest")
pico_set_program_version(mputest "0.1")

# Enable UART output
pico_enable_stdio_uart(mputest 1)

# Enable USB output (set to 1 to use USB, or 0 to disable)
pico_enable_stdio_usb(mputest 1)

# Add the standard library to the build, including hardware_spi for SPI functionality
target_link_libraries(mputest 
        pico_stdlib      # Standard library for Pico
        hardware_spi # SPI for MPU9250
        hardware_i2c # I2C for MS5611 and magnetometer
        hardware_pwm # PWM for motor control
        pico_multicore # Multicore library (if needed in the future)
        pico_sync      # Library for mutex and synchronization primitives
)

# Add include directories if there are header files in other directories
target_include_directories(mputest PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Generate additional output formats (e.g., UF2, ELF, BIN)
pico_add_extra_outputs(mputest)

# Enable binary information output for SPI pins and chip select
pico_enable_stdio_usb(mputest 1)   # Enable USB serial output
pico_enable_stdio_uart(mputest 1)  # Enable UART output if needed
