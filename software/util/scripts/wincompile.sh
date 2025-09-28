#! /bin/bash

# Find compile script working directory
cd "$(dirname "$0")"
COMPILE_WD="$(pwd)/../../build"

# Change to build directory
cd $COMPILE_WD

# Build the code without the Gazebo simulation.
cmake .. -G Ninja -DCOMPILE_SIMULATION=0
ninja
