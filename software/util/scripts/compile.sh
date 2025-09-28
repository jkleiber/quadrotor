#! /bin/bash

# Find compile script working directory
cd "$(dirname "$0")"
SCRIPTS_DIR="$(pwd)"
COMPILE_WD="$(pwd)/../../build"
CONFIG_WD="$(pwd)/../../config"
SIM_SCRIPTS_DIR="$(pwd)/../../../gazebo-sim/scripts"

# Change to build directory
cd $COMPILE_WD

# CMake the quadcopter code, exit on failure
cmake .. -DCOMPILE_SIMULATION=0
if [ $? -ne 0 ]
then
    echo "Error: Quadcopter CMake failed!"
    exit 1
fi

# Make the quadcopter code, exit on failure
make
if [ $? -ne 0 ]
then
    echo "Error: Quadcopter make failed!"
    exit 2
fi

# Copy the current configuration to the binary location, exit on failure
cd $COMPILE_WD
rm -rf $COMPILE_WD/config
cp -R $CONFIG_WD $COMPILE_WD
if [ $? -ne 0 ]
then
    echo "Error: Failed to copy configuration files"
    exit 3
fi

# Compile the simulator plugins
# ${SIM_SCRIPTS_DIR}/compile_sim_plugins.sh
# if [ $? -ne 0 ]
# then
#     echo "Error: Simulator plugins make failed!"
#     exit -1
# fi

