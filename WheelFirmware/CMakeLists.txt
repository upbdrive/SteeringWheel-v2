cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

add_subdirectory(pico-arduino-compat/libs/tinygpsplus)
add_subdirectory(pico-arduino-compat/libs/geniearduino)
add_subdirectory(pico-arduino-compat/libs/mcp2515-nb)


project(WheelFirmware C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(COMPILE_FLAGS "-O3 -Wpedantic")

pico_sdk_init()

add_executable(Firmware
    main.cpp
)
pico_enable_stdio_usb(Firmware 0)
pico_enable_stdio_uart(Firmware 1)


target_include_directories(Firmware PRIVATE pico-arduino-compat/libs/tinygpsplus)
target_link_libraries(Firmware pico_stdlib)
target_link_libraries(Firmware
  pac-tinygpsplus
  pac-geniearduino
  pac-mcp2515-nb
)

pico_add_extra_outputs(Firmware)