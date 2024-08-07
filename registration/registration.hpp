#pragma once
#include <Eigen/Core>
#include <vector>
#include <open3d/Open3D.h>

// Single Pixel struct to represent a tile of the grid map
struct Pixel
{
  // Constructor to create a Pixel from coordinates
  Pixel(int x, int y) : i(x), j(y) {}

  // Constructor to create a Pixel from a point
  Pixel(const Eigen::Vector2d &point, const double &pixel_size)
      : i(static_cast<int>(point[0] / pixel_size)),
        j(static_cast<int>(point[1] / pixel_size)) {}

  // Comparison operator for the Pixel struct
  bool operator==(const Pixel &px) const
  {
    return i == px.i && j == px.j;
  }

  // coordinates of the pixel
  int i;
  int j;
};

// Specialization of std::hash for our custom type Pixel. This will generate unique hash values for each Pixel object
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
namespace registration{
/*
This function takes two pointclouds and alligns them using the ICP algorithm.
input:
  src: the source pointcloud (this pointcloud will be modified!)
  target: the target pointcloud
  pixel_size: the size of the pixels in the grid map
output:
  none
*/
void icp_unknown_correspondences( std::vector<Eigen::Vector2d> &src, const std::vector<Eigen::Vector2d> &target, const double &pixel_size);

/*
This function takes two pointclouds and concatenate them.
input:
  arg1: l_vec - first pointcloud (this pointcloud will be modified!)
  arg2: r_vec - second pointcloud 
output:
  arg1: the concatenated pointcloud vector
*/
std::vector<Eigen::Vector2d> concat_pointclouds(std::vector<Eigen::Vector2d> &l_vec, const std::vector<Eigen::Vector2d> &r_vec);

/*
This function inputs a pointcloud and downsample it with respect to pixel_size and n_points in each pixel.
input:
  arg1: vec - pointcloud to downsample (this pointcloud will be modified!)
  arg2: pixel_size - size for each pixel
  arg3: n_points - number of points to keep in each pixel 
output:
  arg1: downsampled pointcloud vector
*/
std::vector<Eigen::Vector2d> downsample(const std::vector<Eigen::Vector2d> &vec, const double &pixel_size, const int &n_points);


} // namespace registration

