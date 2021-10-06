################################
# GTest
################################
project(googletest-git NONE)

include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        703bd9caab50b139428cea1aaff9974ebee5742e # release-1.10.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

################################
# Tests
################################
add_executable(${TEST_PROJECT_NAME}
    bayesian_filter_tests.cpp
    filter_factory_tests.cpp

    $<TARGET_OBJECTS:bayesian_filters>
    $<TARGET_OBJECTS:filter_factory>
    $<TARGET_OBJECTS:filters>
)

target_link_libraries(${TEST_PROJECT_NAME}
    gtest 
    gtest_main
    
    bayesian_filters    
    filter_factory
    filters
)