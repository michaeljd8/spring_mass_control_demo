#!/bin/bash

# Compile the PlantModel and test files
mkdir -p build
cd build

# Use g++ to compile the source and test files
# Assuming Google Test is installed and available
# Adjust the paths to gtest if necessary
g++ -std=c++17 -I../src/plant_model -I/usr/include/gtest -pthread ../src/plant_model/PlantModel.cpp ../tests/PlantModelTests.cpp -lgtest -lgtest_main -o PlantModelTests

# Run the tests
./PlantModelTests