# Robotics-Project

Made by Massimiliano Roccamena

## Introduction

The aim of the project is to build an odometry computing system of a 4-wheels skid steering robot (Agilex Scout) using ROS middleware.

Gearbox reduction and robot baseline parameters must be calibrated, basing on the ground truth odometry provided by the robot manufacturer.

## The system

The nodes realizing the system are:

- robot (named scout)
- calibration

The entire system can be launched by running

```bash
roslaunch project1 robot_calibration.launch
```

System architecture can be visualized in the following RQT graph

![RQT graph](./project1.png)

Required topics are:

- /scout/twist
- /scout/odom/custom

Required services are:

- /scout/reset
- /scout/set

## Robot

The node can be launched by running

```bash
roslaunch project1 robot.launch
```

The node parameters are:

- /scout/reduction
  - gearbox reduction of the wheels
- /scout/baseline
  - apparent baseline of the robot

(TODO)

## Calibration

The node can be launched by running

```bash
roslaunch project1 calibration.launch
```

The node parameters are:

- /calib/duration
  - time life of the calibration
- /calib/forget
  - parameter of the ema statistic

This node compute a correction, made by some statistics (global average and exponential moving average), of some robot parameters (gearbox reduction and apparent baseline). The corrections are computed during a bag simulation for a given time, and they are a function of manufacturer and robot node odometries (linear and angular velocities). Each value composing a parameter correction is a coefficent to be used as the parameter (estimated) multiplier which minimizes robot node odometry error.
