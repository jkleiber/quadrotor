#! /bin/bash

cd "$(dirname "$0")"

WORLDS="$(pwd)/../../gazebo-sim/worlds"

# export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$(pwd)/models


gazebo ${WORLDS}/quadrotor-blocks.sdf