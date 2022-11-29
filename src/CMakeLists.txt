add_library(pico-ICM20948 ICM20948.cpp)

target_link_libraries(pico-ICM20948 pico_stdlib hardware_i2c)

target_include_directories(pico-ICM20948
    PUBLIC ${PROJECT_SOURCE_DIR}/src
    )

