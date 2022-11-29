add_executable(example main.cpp)

target_link_libraries(example pico-ICM20948 pico_stdlib)

pico_enable_stdio_usb(example 1)
pico_enable_stdio_uart(example 0)

pico_add_extra_outputs(example)