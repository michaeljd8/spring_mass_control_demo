# Minimum CMake version required
cmake_minimum_required(VERSION 3.10)

# Project name
project(SpringMassControlDemo)

# Set C++ standard
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories
include_directories(${CMAKE_SOURCE_DIR})  # Project root for relative includes
include_directories(PlantModel)
include_directories(src)
include_directories(utils)
include_directories(MIL)

# Ensure Ruckig include directory is added to the include paths
if(ruckig_FOUND)
    include_directories(${ruckig_INCLUDE_DIRS})
elseif(RUCKIG_FOUND)
    include_directories(${RUCKIG_INCLUDE_DIRS})
else()
    include_directories(${CMAKE_BINARY_DIR}/_deps/ruckig-src/include)
endif()

# Link libraries
find_package(GTest REQUIRED)
find_package(Threads REQUIRED)

# Find Ruckig library
find_package(ruckig QUIET)
if(NOT ruckig_FOUND)
    # Try to find ruckig using pkg-config as fallback
    find_package(PkgConfig)
    if(PkgConfig_FOUND)
        pkg_check_modules(RUCKIG ruckig)
    endif()
    
    if(NOT RUCKIG_FOUND)
        # Use FetchContent to download and build Ruckig
        include(FetchContent)
        FetchContent_Declare(
            ruckig
            GIT_REPOSITORY https://github.com/pantor/ruckig.git
            GIT_TAG main
        )
        FetchContent_MakeAvailable(ruckig)
    endif()
endif()

# Enable testing framework
enable_testing()

# Custom target to remove CSV files in build directory before building tests
add_custom_target(clean_csv
    COMMAND ${CMAKE_COMMAND} -E echo "Removing .csv files from ${CMAKE_BINARY_DIR}"
    COMMAND ${CMAKE_COMMAND} -E rm -f "${CMAKE_BINARY_DIR}/*.csv"
    COMMENT "Cleaning CSV files from build directory"
)

# Add test for PlantModel_test
add_executable(PlantModel_test tests/PlantModel_test.cpp PlantModel/PlantModel.cpp)
target_link_libraries(PlantModel_test GTest::GTest GTest::Main Threads::Threads)
add_test(NAME PlantModelTest COMMAND PlantModel_test)
add_dependencies(PlantModel_test clean_csv)

# Add test for getters_setters_unit
add_executable(getters_setters_unit tests/getters_setters_unit.cpp src/SpringMassControlDemo.cpp)
target_link_libraries(getters_setters_unit GTest::GTest GTest::Main Threads::Threads ruckig)
add_test(NAME GettersSettersUnitTest COMMAND getters_setters_unit)
add_dependencies(getters_setters_unit clean_csv)

# Add test for velocity profile
add_executable(test_velocity_profile tests/test_velocity_profile.cpp src/SpringMassControlDemo.cpp)
target_link_libraries(test_velocity_profile GTest::GTest GTest::Main Threads::Threads ruckig)
add_test(NAME VelocityProfileTest COMMAND test_velocity_profile)
add_dependencies(test_velocity_profile clean_csv)

# Add test for velocity controller (PID)
add_executable(test_velocity_controller tests/test_velocity_controller.cpp src/SpringMassControlDemo.cpp)
target_link_libraries(test_velocity_controller GTest::GTest GTest::Main Threads::Threads ruckig)
add_test(NAME VelocityControllerTest COMMAND test_velocity_controller)
add_dependencies(test_velocity_controller clean_csv)

# Add MIL Level 1 test
add_executable(MIL_1_test MIL/MIL_L1.cpp PlantModel/PlantModel.cpp src/SpringMassControlDemo.cpp)
target_link_libraries(MIL_1_test ruckig)
add_test(NAME MIL_Level1_Test COMMAND MIL_1_test)
add_dependencies(MIL_1_test clean_csv)

# Add MIL Level 2 test
add_executable(MIL_2_test MIL/MIL_L2.cpp PlantModel/PlantModel.cpp src/SpringMassControlDemo.cpp)
target_link_libraries(MIL_2_test ruckig)
add_test(NAME MIL_Level2_Test COMMAND MIL_2_test)
add_dependencies(MIL_2_test clean_csv)