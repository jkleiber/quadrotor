#! /bin/bash

# Change to scripts directory and run relative to it
cd "$(dirname "$0")"
ROBOT_BIN="$(pwd)/../build"

# Start the gazebo simulation
./launch_blocks_world.sh &

# Change to the robot binaries directory and run the simulator code
cd ${ROBOT_BIN}
./auto_quadrotor_sim