cmake_minimum_required(VERSION 3.16)
project(2d_lidar_odom)

find_package(Open3D REQUIRED HINTS ${PROJECT_SOURCE_DIR}/open3d/lib/cmake)

add_subdirectory(app)
add_subdirectory(dataloader)
add_subdirectory(viewer)
