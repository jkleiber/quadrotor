#! /bin/bash

cd "$(dirname "$0")"

WORLDS="$(pwd)/../../simulation/worlds"

# export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$(pwd)/models


gazebo ${WORLDS}/quadrotor-blocks.sdf