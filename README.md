## 2D Lidar odometry in the Real-World
### Project description

This project implements Iterative closest point algorithm to register consecutive scans taken from 2D-Lidar scanner mounted on a robotic vehicle
This project is aimed to register multiple scans from a 2D-Lidar scanner mounted on a mobile robot vehicle. The scanner, at each instance, takes a full scan of it's surrounding as shown in the Figure 1.

![Single scan](/results/single_scan.png)

### Details

The approach to do this is using iterative closest point algorithm (ICP) to register two consecutive points with eachother and iterate over all scans to get them registered into a single point cloud and get final map of the environment that has been traversed by the robot. The ICP algorithm implemented in this code is from the lecture slides of MSR-II course from Geodetic Engineering at Uni Bonn. The final result looks like shown below.

![Registered scans](/results/registered_scans.png)


## Installation
To run the program, the only external library is open 3d. It has to be extracted in the same directory as these other files. Cmake can be used to build the system and get executables. the most basic commands needed to perform the task are below.
* cmake -Bbuild
* cmake --build build
* ./build/app/main
Note: The last command is to execute the built executables.

## Acknowledgements

- Thanks to [contributors](https://github.com/a-talh/2d_Lidar_odometry/graphs/contributors) for the possibility of this project. 