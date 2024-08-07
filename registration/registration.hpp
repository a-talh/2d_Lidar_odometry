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


// Functions to manipulate point clouds

void icp_unknown_correspondences( std::vector<Eigen::Vector2d> &src, const std::vector<Eigen::Vector2d> &target, const double &pixel_size);

std::vector<Eigen::Vector2d> concat_pointclouds(std::vector<Eigen::Vector2d> &first, const std::vector<Eigen::Vector2d> &second);

std::vector<Eigen::Vector2d> downsample(const std::vector<Eigen::Vector2d> &vec, const double &pixel_size, const int &n_points);