# Minimum CMake version required
cmake_minimum_required(VERSION 3.10)

# Project name
project(SpringMassControlDemo)

# Set C++ standard
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories
include_directories(PlantModel)

# Link libraries
find_package(GTest REQUIRED)
find_package(Threads REQUIRED)

# Enable testing framework
enable_testing()

# Add test for PlantModel_test
add_executable(PlantModel_test tests/PlantModel_test.cpp PlantModel/PlantModel.cpp)
target_link_libraries(PlantModel_test GTest::GTest GTest::Main Threads::Threads)
add_test(NAME PlantModelTest COMMAND PlantModel_test)

# Add test for getters_setters_unit
add_executable(getters_setters_unit tests/getters_setters_unit.cpp src/SpringMassControlDemo.cpp)
target_link_libraries(getters_setters_unit GTest::GTest GTest::Main Threads::Threads)
add_test(NAME GettersSettersUnitTest COMMAND getters_setters_unit)

# Add test for velocity profile
add_executable(test_velocity_profile tests/test_velocity_profile.cpp src/SpringMassControlDemo.cpp)
target_link_libraries(test_velocity_profile GTest::GTest GTest::Main Threads::Threads)
add_test(NAME VelocityProfileTest COMMAND test_velocity_profile)

# Add test for velocity controller (PID)
add_executable(test_velocity_controller tests/test_velocity_controller.cpp src/SpringMassControlDemo.cpp)
target_link_libraries(test_velocity_controller GTest::GTest GTest::Main Threads::Threads)
add_test(NAME VelocityControllerTest COMMAND test_velocity_controller)

# Add Python visualization script as a test
find_package(Python3 REQUIRED)
add_test(NAME PlotPlantTest COMMAND ${Python3_EXECUTABLE} tests/PlantModel_test_vis.py)