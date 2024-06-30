#pragma once
#include <Eigen/Core>
#include <vector>
#include <open3d/Open3D.h>


void viewCloud(const std::vector<Eigen::Vector2d> &pcd);

void viewCloud(const std::vector<Eigen::Vector2d> &first, const std::vector<Eigen::Vector2d> &second);

std::pair<Eigen::Matrix2d, Eigen::Vector2d> implement_icp(std::vector<Eigen::Vector2d> &src, const std::vector<Eigen::Vector2d> &target);

std::vector<Eigen::Vector2d> apply_transformation(const std::pair <Eigen::Matrix2d, Eigen::Vector2d> &transformation, const std::vector<Eigen::Vector2d> &src);