add_library(ICM20948 ICM20948.cpp)

target_link_libraries(ICM20948 pico_stdlib hardware_i2c)

target_include_directories(ICM20948
    PUBLIC ${PROJECT_SOURCE_DIR}/src
    )

