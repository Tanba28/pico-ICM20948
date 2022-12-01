cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(ICM20948 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_compile_options(
        -Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
)

add_subdirectory(./src)

if(NOT DEFINED IS_ROOT_PROJECT)
        set(IS_ROOT_PROJECT true)
        add_subdirectory(./example)
        add_subdirectory(./pico-FreeRTOS)
        add_subdirectory(./pico-I2C)
endif()