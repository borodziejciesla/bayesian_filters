add_subdirectory(matplotlib_cpp)

add_executable(${EXAMPLE_PROJECT_NAME}
    example.cpp
    matplotlib_cpp/matplotlibcpp.h

    $<TARGET_OBJECTS:bayesian_filters>
)

find_package(Python3 COMPONENTS Interpreter Development)
if (${Python3_FOUND})
  include_directories(${Python3_INCLUDE_DIRS})
else()
  message(FATAL_ERROR "Python3 not found, please install it.")
endif()

include_directories(${EXAMPLE_PROJECT_NAME}
    PUBLIC
        matplotlib_cpp
        ${Python3_INCLUDE_DIRS}
)

target_link_libraries(${EXAMPLE_PROJECT_NAME}
    bayesian_filters
    ${Python3_LIBRARIES}
)