cmake_minimum_required(VERSION 3.13)

# Include the Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Project configuration
project(pico_cap1188 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK
pico_sdk_init()

# Library source files
set(CAP1188_SOURCES
    src/cap1188.cpp
)

# Library headers
set(CAP1188_HEADERS
    include/cap1188/cap1188.hpp
    include/cap1188/cap1188_registers.hpp
    include/cap1188/cap1188_types.hpp
)

# Create the CAP1188 library
add_library(cap1188 STATIC ${CAP1188_SOURCES})

# Include directories for the library
target_include_directories(cap1188 PUBLIC include)

# Link Pico SDK libraries to our library
target_link_libraries(cap1188 
    pico_stdlib
    hardware_i2c
    hardware_gpio
)

# Set compile options for the library
target_compile_options(cap1188 PRIVATE 
    -Wall 
    -Wextra 
    -Werror
    -Wno-unused-parameter
)

# Enable USB output for printf debugging (optional)
# Uncomment the following lines to enable USB stdio
# pico_enable_stdio_usb(cap1188 1)
# pico_enable_stdio_uart(cap1188 0)

# Basic touch example
add_executable(basic_touch examples/basic_touch/basic_touch.cpp)

# Link libraries to the example
target_link_libraries(basic_touch 
    cap1188
    pico_stdlib
    hardware_i2c
    hardware_gpio
)

# Enable USB and UART output for the example
pico_enable_stdio_usb(basic_touch 1)
pico_enable_stdio_uart(basic_touch 1)

# Create map/bin/hex/uf2 files for the example
pico_add_extra_outputs(basic_touch)

# Installation rules (optional)
install(TARGETS cap1188
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.hpp"
)

# Package configuration (optional)
set(CAP1188_VERSION "0.0.1")
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cap1188-config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cap1188-config.cmake"
    @ONLY
)

# Add compile definitions
target_compile_definitions(cap1188 PRIVATE
    CAP1188_VERSION_MAJOR=0
    CAP1188_VERSION_MINOR=0
    CAP1188_VERSION_PATCH=1
)

# Add version information to the example
target_compile_definitions(basic_touch PRIVATE
    CAP1188_VERSION_MAJOR=0
    CAP1188_VERSION_MINOR=0
    CAP1188_VERSION_PATCH=1
)