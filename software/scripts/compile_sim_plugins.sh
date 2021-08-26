#! /bin/bash

cd "$(dirname "$0")"
PLUGINS_BUILD_DIR="$(pwd)/../../simulation/plugins/build"

cd $PLUGINS_BUILD_DIR

# CMake simulator plugins, exit on failure
cmake ..
if [ $? -ne 0 ]
then
    echo "Error: Simulator plugins CMake failed!"
    exit 4
fi

# Make the simulator plugins, exit on failure
make
if [ $? -ne 0 ]
then
    echo "Error: Simulator plugins make failed!"
    exit 5
fi