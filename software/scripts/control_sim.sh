#! /bin/bash

# Change to scripts directory and run relative to it
cd "$(dirname "$0")"
CONTROL_SIM_BIN="$(pwd)/../build/src/control_sim"


# Change to the robot binaries directory and run the simulator code
cd ${CONTROL_SIM_BIN}
./control_sim