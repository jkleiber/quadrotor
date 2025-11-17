#! /bin/bash

# Change to scripts directory and run relative to it
cd "$(dirname "$0")"
BIN_DIR="$(pwd)/../../build/apps"


# Change to the robot binaries directory and run the simulator code
cd ${BIN_DIR}
./quadcopter_simulator