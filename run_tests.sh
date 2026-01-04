#!/bin/bash

# Variables
BUILD_DIR="build"
TEST_EXEC="plant_model_test"
SRC_DIR="src/plant_model"
TEST_DIR="tests"

# Create build directory if it doesn't exist
mkdir -p $BUILD_DIR

# Compile the test

g++ -std=c++17 -I$SRC_DIR -o $BUILD_DIR/$TEST_EXEC $TEST_DIR/plant_model_test.cpp $SRC_DIR/plant_model.cpp -lgtest -lpthread

# Check if compilation was successful
if [ $? -eq 0 ]; then
    echo "Compilation successful. Running tests..."
    ./$BUILD_DIR/$TEST_EXEC
else
    echo "Compilation failed."
    exit 1
fi

# Run visualization script
python3 tests/plot_velocity.py