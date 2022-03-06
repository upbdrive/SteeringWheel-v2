cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

add_subdirectory(../extlibs/pico-arduino-compat/libs/tinygpsplus build/tinygpsplus)
add_subdirectory(../extlibs/pico-arduino-compat/libs/geniearduino build/geniearduino)
add_subdirectory(../extlibs/pico-arduino-compat/libs/mcp2515-nb build/mcp2515)
add_subdirectory(../extlibs/pico-arduino-compat/libs/acan2515 build/acan2515)
add_subdirectory(../extlibs/pico-arduino-compat/libs/acan build/acan)


project(WheelFirmware C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(COMPILE_FLAGS "-O3 -Wpedantic")

pico_sdk_init()

add_executable(Firmware
    main.cpp
)

pico_enable_stdio_usb(Firmware 0)
pico_enable_stdio_uart(Firmware 0)


target_include_directories(Firmware PRIVATE pico-arduino-compat/libs/tinygpsplus)
target_link_libraries(Firmware pico_stdlib)
target_link_libraries(Firmware
  pac-tinygpsplus
  pac-geniearduino
  pac-mcp2515-nb
  pico_multicore
)
pico_add_extra_outputs(Firmware)


add_executable(CANTest
    ECUTest.cpp
)

target_include_directories(CANTest PRIVATE pico-arduino-compat/libs/tinygpsplus)
target_link_libraries(CANTest pico_stdlib hardware_spi)
target_link_libraries(CANTest
  pac-tinygpsplus
  pac-geniearduino
  pac-mcp2515-nb
  pico_multicore
)

pico_enable_stdio_usb(CANTest 1)
pico_enable_stdio_uart(CANTest 0)
pico_add_extra_outputs(CANTest)