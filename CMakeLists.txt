cmake_minimum_required(VERSION 3.13)

project(soil_modbus_rtu C)

add_library(soil_modbus_rtu STATIC
    src/soil_modbus_rtu.c
)

target_include_directories(soil_modbus_rtu
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
