
# QuadrotorSim

A simulator for the quadrotor to operate in a more real-world environment.



## Getting Started

### Prerequisites
- Unreal Engine 5
- Must manually install these UE5 Plugins (to the Engine/Plugins/Marketplace directory):
    - [Cesium for Unreal v2.19.1](https://github.com/CesiumGS/cesium-unreal/releases/tag/v2.19.1)

### Installation
After installing the above prerequisites, simply run:
```
. ./setup.sh PATH_TO_UNREAL_ENGINE
```

where `PATH_TO_UNREAL_ENGINE` is your path to the folder containing the Unreal Engine `Engine/` folder. This script will automatically update submodules, install dependencies, build the project, and open it in the Unreal Engine Editor. 

## Development

After initial setup, you can rebuild the project with `./edit.sh` (which will also open the project in the Unreal Engine Editor).

