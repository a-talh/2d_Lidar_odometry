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

private:
  std::filesystem::path data_root_dir_;
  std::vector<std::string> laser_scan_files_;
};
} // namespace dataset
