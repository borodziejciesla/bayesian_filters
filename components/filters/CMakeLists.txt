cmake_minimum_required(VERSION 3.13)

add_library(filters OBJECT
    src/core_bayesian_filter.cpp
    src/kalman_filter.cpp
    src/extended_kalman_filter.cpp
    src/unscented_kalman_filter.cpp
)

target_include_directories(filters
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/include/io
        $<TARGET_PROPERTY:eigen,INTERFACE_INCLUDE_DIRECTORIES>
)