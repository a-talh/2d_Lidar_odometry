
# 2D Lidar odometry in the Real-World

- [2D Lidar odometry in the Real-World](#2d-lidar-odometry-in-the-real-world)
  - [Project description](#project-description)
  - [Implementation details](#implementation-details)
  - [Execution time](#execution-time)
  - [Installation](#installation)
    - [Ubuntu](#ubuntu)
  - [Acknowledgements](#acknowledgements)

## Project description

This project implements Iterative closest point algorithm to register consecutive scans taken from 2D-Lidar scanner mounted on a robotic vehicle
This project is aimed to register multiple scans from a 2D-Lidar scanner mounted on a mobile robot vehicle. The scanner, at each instance, takes a full scan of it's surrounding as shown in image below.  

![Single scan](/results/single_scan.png)

## Implementation details

The approach to do this is using iterative closest point algorithm (ICP) to register two consecutive points with eachother and iterate over all scans to get them registered into a single point cloud and get final map of the environment that has been traversed by the robot. The ICP algorithm implemented in this code is from the lecture slides of MSR-II course from Geodetic Engineering at Uni Bonn. The final result looks like shown below. The goal is to register scans at minimum 10Hz.  

![Registered scans](/results/registered_scans.png)

## Execution time  

Evaluation results will be uploaded later

## Installation  

### Ubuntu  

Firstly, clone this repository into any folder of your choice. To run the program, the external libraries needed are Eigen3d and Open3d. Eigen3d can be installed using following command in terminal.  

- `sudo apt install libeigen3-dev`  

Once successfully installed, Open3d has to be downloaded from github and extracted to our project folder. Open3d can be downloaded from [Open3d github](https://github.com/isl-org/Open3D/releases). For ubuntu system, download this one _open3d-devel-linux-x86_64-cxx11-abi-0.18.0.tar.xz_ from the releases. Extract it to project directory and rename the folder as "open3d". After extracting, project directory looks like below. Note that you have to add data folder yourself as it isn't shared here. 

![directory tree](/results/tree.png)  

Cmake can be used to build the system and get executables. the most basic commands needed to perform the task are below.  

- `cmake -Bbuild`
- `cmake --build build`
- `./build/app/main`  

_Note: Once the build is successful using first two commands, third command can be used to execute our built binaries._

## Acknowledgements  

Thanks to [contributors](https://github.com/a-talh/2d_Lidar_odometry/graphs/contributors) for the possibility of this project.