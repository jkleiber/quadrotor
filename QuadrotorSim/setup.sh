#!/bin/bash

UNREAL_PATH="${1:-~/apps/Linux_Unreal_Engine_5.6.1}"

# Run script from the script's location.
ORIGINAL_DIR=$(pwd)

# Tempo pre-requisites.
sudo apt update
sudo apt install -y curl jq

if [ -z $UNREAL_ENGINE_PATH ]; then
    echo "UNREAL_ENGINE_PATH not set. Setting to $UNREAL_PATH in ~/.bashrc of current shell."
    echo "UNREAL_ENGINE_PATH=$UNREAL_PATH" >> ~/.bashrc
    export UNREAL_ENGINE_PATH=$UNREAL_PATH
    . ~/.bashrc
fi

# Update submodules to pull in project-specific plugins.
git submodule update --init --recursive

# Tempo initialization.
cd Plugins
./Tempo/Setup.sh

./Tempo/Scripts/Build.sh
./Tempo/Scripts/Run.sh

# Restore the original shell.
cd $ORIGINAL_DIR