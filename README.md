# Robotics-Project

Made by Massimiliano Roccamena

## Introduction

The aim of the project is to build an odometry computing system of a 4-wheels skid steering robot (Agilex Scout) using ROS middleware.

Gearbox reduction and apparent baseline parameters must be calibrated, basing on the ground truth odometry provided by the robot manufacturer.

## Overview

The nodes realizing the system are:

- robot (named scout)
- calibration

The entire system can be launched by running

```bash
roslaunch project1 robot_calibration.launch
```

System architecture can be visualized in the following RQT graph

![RQT graph](./project1.png)

## Requirements

Required topics are:

- /scout/twist
- /scout/odom/basic
- /scout/odom/custom (with integration method)

Required services are:

- /scout/reset (to 0,0)
- /scout/set

Required dynamic parameter is intMethod for node robot (/scout)

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

This node is in charge of computing the odometry (and other required functionalities) of the robot based on the system inputs, which are the 4 wheels rpms provided; the odometry can be computed using Euler and Runge-Kutta integration methods.

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

This node compute a set of corrections, each of which is an error indicator of a robot node parameter; each correction is realized by the following set of statistics of the indicator:

- global average
- exponential moving average (used for visualization of instantaneous values, with high frequencies filtered)

The error indicators are computed for a given time with respect to a bag and robot node simulation, and they are a function of manufacturer and robot node odometries (twist fields):

- Gearbox reduction coefficent
  - k1 = V_robot / V_manufacturer = reduction_manufacturer / reduction_robot
- Apparent baseline coefficent
  - k2 = W_robot / W_manufacturer / k1 = reduction_manufacturer / reduction_robot * baseline_manufacturer / baseline_robot / k1 = base_manufacturer / base_robot

Each indicator is a coefficent to be used as the parameter (estimated) multiplier which maximizes robot node odometry similarity with manufacturer one.
