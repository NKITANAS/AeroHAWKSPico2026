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
# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == DO NOT EDIT (VS Code extension requirement) ==
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
# ==================================================

set(PICO_BOARD pico2_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(AeroHAWKS2026 C CXX ASM)

set(PICO_CXX_ENABLE_EXCEPTIONS 1)
set(PICO_CXX_ENABLE_RTTI 1)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Collect all .cpp files from src/cpp
file(GLOB_RECURSE AEROHAWKS2026_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/src/cpp/*.cpp"
)

# Create executable
add_executable(AeroHAWKS2026
    ${AEROHAWKS2026_SOURCES}
)

pico_set_program_name(AeroHAWKS2026 "AeroHAWKS2026")
pico_set_program_version(AeroHAWKS2026 "0.1")

# Enable or disable USB/UART stdio
pico_enable_stdio_uart(AeroHAWKS2026 0)
pico_enable_stdio_usb(AeroHAWKS2026 1)

# Libraries to link
target_link_libraries(AeroHAWKS2026
    pico_stdlib
    hardware_i2c
)

# Add include directory
target_include_directories(AeroHAWKS2026 PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src/include
)

# Add any extra user libraries below
target_link_libraries(AeroHAWKS2026
    # your additional libraries here
)

# Generate UF2, bin, and map files
pico_add_extra_outputs(AeroHAWKS2026)
