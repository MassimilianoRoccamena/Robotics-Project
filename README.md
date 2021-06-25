# Robotics-Project

Name: Massimiliano Roccamena

Person Code: 10499005

## Introduction

The objective of the project is to build a mapping system and a localization system for a 4-wheels skid steering robot (Agilex Scout) using ROS middleware.

## Overview

The basic components of the system are:

- fix
- imu_filter
- sensor_fusion
- filtering
- mapping
- localization

Mapping system can be launched by running:

```bash
roslaunch project1 scout_mapping.launch
```

Here are plotted the RQT graph and TF tee:

![RQT graph](./imgs/graph-mapping.png)
![TF tree](./imgs/tree-mapping.png)

Localization system can be launched by running

```bash
roslaunch project1 scout_localization.launch
```

Here are plotted the RQT graph and TF tee:

![RQT graph](./imgs/graph-localization.png)
![TF tree](./imgs/tree-localization.png)

## fix

In this component two functionalities are executed:

- fix_scout_odom and fix_camera_odom nodes re-send robot and visual odometries topics by changing the TFs into the target frames (/odom and /baselink frames) for later sensor fusion; involved topics are:
  - /odom		 ->  /scout/odom
  - /camera/odom/sample  ->  /camera/odom
- some static TF publishers fill missing TFs, in particular /laser frame is linked to /base_link with a rotation of 120°.

## imu_data

Filtering of raw IMU data

## sensor_fusion

Here we fuse all our sensors (robot odometry, visual odometry, IMU) and create an odometry filter with robot_localization EKF node; to avoid oscillations of pose (positions and angle) only scout odometry is involved in position estimation, and only IMU is involved in angle estimation; in this way visual odometry can be joined in the filter in differential mode in all odometry variables.

## filtering

Execution of IMU filtering and sensor fusion.

## mapping

Basic mapping system based on gmapping laser scan SLAM.

## localization

Basic Monte Carlo localization based on amcl.

! ATTENTION ! loading of the map is commented because for some reason on my machine map_server fail to load any map from launch file; either try to uncomment this or load manually the map (map1 in /maps path is created from bag1).

## scout_mapping / scout_localization

Mapping / localization systems with fixed TFs and filtered odometries.