#pragma once
#include <Eigen/Core>
#include <vector>
#include <open3d/Open3D.h>


void viewCloud(const std::vector<Eigen::Vector2d> &pcd);

void viewCloud(const std::vector<Eigen::Vector2d> &first, const std::vector<Eigen::Vector2d> &second);

std::pair<Eigen::Matrix2d, Eigen::Vector2d> icp_known_correspondence(std::vector<Eigen::Vector2d> &src, const std::vector<Eigen::Vector2d> &target);

std::pair<Eigen::Matrix2d, Eigen::Vector2d> icp_unknown_correspondence(std::vector<Eigen::Vector2d> &src, const std::vector<Eigen::Vector2d> &target);

std::vector<Eigen::Vector2d> apply_transformation(const std::pair <Eigen::Matrix2d, Eigen::Vector2d> &transformation, const std::vector<Eigen::Vector2d> &src);

std::vector<Eigen::Vector2d> sample_points(const std::vector<Eigen::Vector2d> &src, int num_samples);

std::vector<Eigen::Vector2d> find_nearest_neighbours(const std::vector<Eigen::Vector2d> &src, const std::vector<Eigen::Vector2d> &target);

std::vector<Eigen::Vector2d> concat_pointclouds(const std::vector<Eigen::Vector2d> &first, const std::vector<Eigen::Vector2d> &second);