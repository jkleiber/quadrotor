#! /bin/bash

# Change to scripts directory and run relative to it
cd "$(dirname "$0")"
ROBOT_BIN="$(pwd)/../build/apps"
SIM_BIN="$(pwd)/../../gazebo-sim/plugins/build"

# Start the gazebo simulation
cd ${SIM_BIN}
./launch_blocks_world.sh &

# Change to the robot binaries directory and run the simulator code
cd ${ROBOT_BIN}
./quadrotor_fc_sim