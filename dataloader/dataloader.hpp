#pragma once

#include <Eigen/Core>
#include <filesystem>
#include <string>
#include <vector>

namespace dataset {
class LaserScanDataset {
public:
  using PointCloud = std::vector<Eigen::Vector2d>;

  LaserScanDataset(const std::string &data_root_dir);
  std::size_t size() const { return laser_scan_files_.size(); }
  [[nodiscard]] PointCloud operator[](int idx) const;

  void SetRegisteredPointCloud(const PointCloud &pointcloud);
  void AddRegisteredPointCloud(const PointCloud &pointcloud);
  [[nodiscard]] PointCloud GetRegisteredPointCloud() const;

  void SetTransformation(std::pair<Eigen::Matrix2d, Eigen::Vector2d> &transformation);
  [[nodiscard]] std::pair<Eigen::Matrix2d, Eigen::Vector2d> GetTransformation(int idx) const;

private:
  std::filesystem::path data_root_dir_;
  std::vector<std::string> laser_scan_files_;
  PointCloud registered_pointcloud_;
  std::vector<Eigen::Vector2d> t_;
  std::vector<Eigen::Matrix2d> R_;
};
} // namespace dataset
