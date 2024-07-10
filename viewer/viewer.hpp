#pragma once
#include <Eigen/Core>
#include <vector>
#include <open3d/Open3D.h>


void viewCloud(const std::vector<Eigen::Vector2d> &pcd);

void viewCloud(const std::vector<Eigen::Vector2d> &first, const std::vector<Eigen::Vector2d> &second);

std::vector<Eigen::Vector3d> get_points(const open3d::geometry::PointCloud &pcd);

Eigen::Matrix3d icp_known_corres(std::vector<Eigen::Vector2d> &src, const std::vector<Eigen::Vector2d> &target);

Eigen::Matrix3d icp_unknown_correspondence(std::vector<Eigen::Vector2d> &src, const std::vector<Eigen::Vector2d> &target);

std::vector<Eigen::Vector2d> apply_transformation(const Eigen::Matrix3d &transformation, const std::vector<Eigen::Vector2d> &src);

std::vector<Eigen::Vector2d> concat_pointclouds(std::vector<Eigen::Vector2d> &first, const std::vector<Eigen::Vector2d> &second);