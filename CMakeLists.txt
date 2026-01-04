# Minimum CMake version required
cmake_minimum_required(VERSION 3.10)

# Project name
project(SpringMassControlDemo)

# Set C++ standard
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories
include_directories(PlantModel)

# Add the executable for the test
add_executable(PlantModel_test tests/PlantModel_test.cpp PlantModel/PlantModel.cpp)

# Link libraries
find_package(GTest REQUIRED)
find_package(Threads REQUIRED)

target_link_libraries(PlantModel_test GTest::GTest GTest::Main Threads::Threads)

# Add a custom target to run the tests
add_custom_target(run_tests
    COMMAND $<TARGET_FILE:PlantModel_test>
    DEPENDS PlantModel_test
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Add a custom target to run the Python visualization script
add_custom_target(plot_plant_test
    COMMAND ${CMAKE_COMMAND} -E env python3 tests/PlantModel_test_vis.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

# Add the executable for the getters and setters test
add_executable(getters_setters_unit tests/getters_setters_unit.cpp src/SpringMassControlDemo.cpp)

# Link libraries for the getters and setters test
target_link_libraries(getters_setters_unit GTest::GTest GTest::Main Threads::Threads)

# Add a custom target to run the getters and setters test
add_custom_target(run_getters_setters_unit
    COMMAND $<TARGET_FILE:getters_setters_unit>
    DEPENDS getters_setters_unit
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Add a custom target to build and run the getters and setters test
add_custom_target(build_and_run_getters_setters_unit
    COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target getters_setters_unit
    COMMAND $<TARGET_FILE:getters_setters_unit>
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)