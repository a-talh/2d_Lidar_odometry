#pragma once
#include <Eigen/Core>
#include <vector>
#include <open3d/Open3D.h>

void viewCloud(const std::vector<Eigen::Vector2d> &pcd);

void viewCloud(const std::vector<Eigen::Vector2d> &first, const std::vector<Eigen::Vector2d> &second);

void implement_icp(std::vector<Eigen::Vector2d> &src, const std::vector<Eigen::Vector2d> &target);