#pragma once
#include <Eigen/Core>
#include <vector>
#include <open3d/Open3D.h>

// Single Pixel struct to represent a tile of the grid map
struct Pixel
{
  Pixel(int x, int y) : i(x), j(y) {}

  Pixel(const Eigen::Vector2d &point, const double &pixel_size)
      : i(static_cast<int>(point[0] / pixel_size)),
        j(static_cast<int>(point[1] / pixel_size)) {}

  bool operator==(const Pixel &px) const
  {
    return i == px.i && j == px.j;
  }

  int i;
  int j;
};

// Specialization of std::hash for our custom type Pixel
namespace std
{

  template <>
  struct hash<Pixel>
  {
    size_t operator()(const Pixel &px) const
    {
      return ((1 << 20) - 1) & (px.i * 73856093 ^ px.j * 19349663);
    }
  };

} // namespace std

// Functions to view a point cloud
void viewCloud(const std::vector<Eigen::Vector2d> &pcd);
void viewCloud(const std::vector<Eigen::Vector2d> &first, const std::vector<Eigen::Vector2d> &second);

// Functions to manipulate point clouds
std::vector<Eigen::Vector3d> get_points(const open3d::geometry::PointCloud &pcd);

Eigen::Matrix3d icp_known_correspondence(std::vector<Eigen::Vector2d> &src, const std::vector<Eigen::Vector2d> &target);

std::unordered_map<Pixel, std::vector<Eigen::Vector2d>> grid_map(const std::vector<Eigen::Vector2d> &vec, double pixel_size);

std::tuple<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>>
nearest_neighbours(const std::vector<Eigen::Vector2d> &src, const std::unordered_map<Pixel, std::vector<Eigen::Vector2d>> &target_grid, const double &pixel_size);

Eigen::Matrix3d icp_unknown_correspondence(const std::vector<Eigen::Vector2d> &src, const std::vector<Eigen::Vector2d> &target, const double &pixel_size);

std::vector<Eigen::Vector2d> apply_transformation(const Eigen::Matrix3d &transformation, const std::vector<Eigen::Vector2d> &src);

std::vector<Eigen::Vector2d> concat_pointclouds(std::vector<Eigen::Vector2d> &first, const std::vector<Eigen::Vector2d> &second);

std::vector<Eigen::Vector2d> downsample(const std::vector<Eigen::Vector2d> &vec, const double &pixel_size, const int &n_points);

std::vector<Eigen::Vector2d> downSampleMean(const std::vector<Eigen::Vector2d> &vec, const double &pixel_size);
