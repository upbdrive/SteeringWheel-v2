cmake_minimum_required(VERSION 3.13)

include(../extlibs/pico-sdk/pico_sdk_init.cmake)

project(WheelFirmware C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(COMPILE_FLAGS "-O2 -Wpedantic")

pico_sdk_init()

add_executable(Firmware
    main.cpp
    ../extlibs/genie/src/geniePicoDEV.cpp
    ../extlibs/mcp2515/src/MCP2515.cpp
    ../extlibs/mcp2515/src/CANPacket.cpp
    ../extlibs/mcp2515/src/Utilities.cpp
)

pico_enable_stdio_usb(Firmware 1)
pico_enable_stdio_uart(Firmware 0)


#target_include_directories(Firmware PRIVATE ../extlibs/genie/src ../extlibs/libgenie/src)
target_include_directories(Firmware PRIVATE ../extlibs/genie/src ../extlibs/mcp2515/src)
target_link_libraries(Firmware pico_stdlib)
target_link_libraries(Firmware
  pico_multicore
  hardware_spi
)
pico_add_extra_outputs(Firmware)


#add_executable(CANTest
##    ECUTest.cpp
#)

##target_include_directories(CANTest PRIVATE pico-arduino-compat/libs/tinygpsplus)
#target_link_libraries(CANTest pico_stdlib hardware_spi)
#target_link_libraries(CANTest
#  pac-tinygpsplus
##  pac-geniearduino
#  pac-mcp2515-nb
#  pico_multicore
#)

#pico_enable_stdio_usb(CANTest 1)
#pico_enable_stdio_uart(CANTest 0)
#pico_add_extra_outputs(CANTest)