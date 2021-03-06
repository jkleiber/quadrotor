#! /bin/bash

cd "$(dirname "$0")"
WORLDS="$(pwd)/../../simulation/worlds"
PLUGINS_BUILD_DIR="$(pwd)/../../simulation/plugins/build"

# Need to be in the plugin directory
cd $PLUGINS_BUILD_DIR

# Add the plugin to the gazebo path
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$(pwd)

# Launch the world
gazebo --verbose ${WORLDS}/quadrotor-blocks.sdf
