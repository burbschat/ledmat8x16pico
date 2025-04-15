cmake_minimum_required(VERSION 3.13)

set(PICO_SDK_PATH "../../pico-sdk")

include(pico_sdk_import.cmake)

project(ledmat8x16pico_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

# Declare executable
add_executable(ledmat8x16pico)

# Add sources to executable
target_sources(ledmat8x16pico PRIVATE ledmat8x16pico.c)

# Generate header from pio assembly file
pico_generate_pio_header(ledmat8x16pico ${CMAKE_CURRENT_LIST_DIR}/tlc59283.pio)


# Enabel stdio output via USB and UART
pico_enable_stdio_usb(ledmat8x16pico 1)
pico_enable_stdio_uart(ledmat8x16pico 1)

# Enable extra outputs
pico_add_extra_outputs(ledmat8x16pico)

# Set libraries to link against
target_link_libraries(ledmat8x16pico PRIVATE pico_stdlib hardware_pio hardware_dma)
