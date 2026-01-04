# Minimum CMake version required
cmake_minimum_required(VERSION 3.10)

# Project name
project(SpringMassControlDemo)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories
include_directories(plant_model)

# Add the executable for the test
add_executable(plant_model_test tests/plant_model_test.cpp plant_model/plant_model.cpp)

# Link libraries
find_package(GTest REQUIRED)
find_package(Threads REQUIRED)

target_link_libraries(plant_model_test GTest::GTest GTest::Main Threads::Threads)

# Add a custom target to run the tests
add_custom_target(run_tests
    COMMAND $<TARGET_FILE:plant_model_test>
    DEPENDS plant_model_test
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Add a custom target to run the Python visualization script
add_custom_target(plot_velocity
    COMMAND ${CMAKE_COMMAND} -E env python3 tests/plot_velocity.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)