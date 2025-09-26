
# Quadrotor

This is a project for the hardware and software of a quadcopter.

## Quadcopter 3D CAD
The quadcopter is 3D printed and all CAD work is done in OnShape in [this project](https://cad.onshape.com/documents/413384f17bd393d247c9b79e/w/f46ad751ac408daab1c55c23/e/428b8b34f12af24e5b48da17?renderMode=0&uiState=62fc49f3bee5066f5bad792f)

![Quadcopter CAD](/img/quadcopter-cad.png)

## Control Simulation GUI

The control simulator uses ImGui to allow for real-time data viewing and parameter tuning. Below is a snapshot of the GUI:

![Control Sim GUI](/img/gui_example.png)

## Hardware

A custom flight controller made in KiCAD exists in the `hardware/` directory. This is a breakout board for a Teensy, BNO055 IMU, a GPS module and the PWM outputs for the ESCs.

## Gazebo Simulation

There is a gazebo simulation environment in the `gazebo-sim/` directory. It is somewhat functional, but more work has been put into the control simulation GUI. The current gazebo settings aren't configured for the actual drone.
