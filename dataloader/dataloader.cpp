#include "dataloader.hpp"

#include <Eigen/Core>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

std::vector<std::string> ReadLaserScanFiles(const fs::path &laserscan_path) {
  std::vector<std::string> filenames;
  for (const auto &file : fs::directory_iterator(laserscan_path)) {
    if (file.path().extension() == ".bin") {
      filenames.emplace_back(file.path().string());
    }
  }
  if (filenames.empty()) {
    std::cerr << laserscan_path << "contains no files with .bin extension"
              << std::endl;
    exit(1);
  }
  std::sort(filenames.begin(), filenames.end());

  return filenames;
}

std::vector<Eigen::Vector2d> ReadLaserScan(const std::string &filename) {
  std::ifstream laserscan_file(filename.c_str(), std::ios::binary);

  laserscan_file.seekg(0, std::ios::end);
  uint32_t num_points = laserscan_file.tellg() / (3 * sizeof(float));
  laserscan_file.seekg(0, std::ios::beg);

  std::vector<float> values(3 * num_points);
  laserscan_file.read((char *)&values[0], 3 * num_points * sizeof(float));
  laserscan_file.close();

  std::vector<Eigen::Vector2d> pointcloud;
  pointcloud.resize(num_points);
  for (uint32_t i = 0; i < num_points; i++) {
    pointcloud[i].x() = values[i * 3];
    pointcloud[i].y() = values[i * 3 + 1];
  }
  return pointcloud;
}
} // namespace

namespace dataset {
LaserScanDataset::LaserScanDataset(const std::string &data_root_dir) {
  data_root_dir_ = fs::absolute(fs::path(data_root_dir));
  laser_scan_files_ = ReadLaserScanFiles(data_root_dir_ / "BINARY/");
}

LaserScanDataset::PointCloud LaserScanDataset::operator[](int idx) const {
  return ReadLaserScan(laser_scan_files_[idx]);
}
} // namespace dataset
