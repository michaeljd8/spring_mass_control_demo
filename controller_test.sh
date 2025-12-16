#!/bin/bash

# This script compiles and runs the unit tests for the Controller class.

# Variables
BUILD_DIR="build"
TEST_EXEC="controller_test"
SRC_DIR="src/control"
TEST_DIR="tests"

# Create build directory if it doesn't exist
mkdir -p $BUILD_DIR

# Compile the test
g++ -std=c++17 -I$SRC_DIR -o $BUILD_DIR/$TEST_EXEC $TEST_DIR/controller_test.cpp $SRC_DIR/controller.cpp -lgtest -lpthread

# Check if compilation was successful
if [ $? -eq 0 ]; then
    echo "Compilation successful. Running tests..."
    ./$BUILD_DIR/$TEST_EXEC
else
    echo "Compilation failed."
    exit 1
fi