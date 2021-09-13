cmake_minimum_required(VERSION 3.13)

add_library(filter_factory
    src/bayesian_filter_factory.cpp
)

target_include_directories(filter_factory
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/include/io
        $<TARGET_PROPERTY:filters,INTERFACE_INCLUDE_DIRECTORIES>
)