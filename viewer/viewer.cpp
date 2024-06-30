#include "viewer.hpp"
#include <algorithm>
#include <iostream>
#include <open3d/Open3D.h>
#include <open3d/core/ShapeUtil.h>
#include <utility>
#include <vector>

namespace
{
    Eigen::Vector2d compute_mean(const std::vector<Eigen::Vector2d> &vec)
    {
        Eigen::Vector2d zero = Eigen::Vector2d::Zero();
        Eigen::Vector2d mean = std::accumulate(vec.begin(), vec.end(), zero);
        return mean / vec.size();
    }

    Eigen::Matrix2d compute_covariance(const std::vector<Eigen::Vector2d> &vec1, const std::vector<Eigen::Vector2d> &vec2, const Eigen::Vector2d &mean1, const Eigen::Vector2d &mean2)
    {
        Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();

        for (size_t i = 0; i < vec1.size(); i++)
        {
            cov += (vec2[i] - mean2) * (vec1[i] - mean1).transpose();
        };

        return cov;
    }
}

void viewCloud(const std::vector<Eigen::Vector2d> &pcd) {
  std::vector<Eigen::Vector3d> pts(pcd.size());
  std::transform(pcd.cbegin(), pcd.cend(), pts.begin(), [](const auto &p) {
    return Eigen::Vector3d(p.x(), p.y(), 0.0);
  });
  open3d::geometry::PointCloud pointcloud{pts};
  pointcloud.PaintUniformColor(Eigen::Vector3d(1.0, 0.0, 0.0));
  open3d::visualization::DrawGeometries(
      {std::make_shared<open3d::geometry::PointCloud>(pointcloud)});
}

void viewCloud(std::vector<Eigen::Vector2d> &first, const std::vector<Eigen::Vector2d> &second)
{
    first.reserve(first.size() + second.size());
    first.insert(first.end(), second.begin(), second.end());
    viewCloud(first);

}


void implement_icp(std::vector<Eigen::Vector2d> &src, const std::vector<Eigen::Vector2d> &target)
{   
    std::cout << "Implementing ICP" << std::endl;
    Eigen::Vector2d x0 = Eigen::Vector2d::Zero();
    Eigen::Vector2d y0 = Eigen::Vector2d::Zero();
    Eigen::Matrix2d H = Eigen::Matrix2d::Zero();

    for (int i  = 0; i <= 5; i++)
    {
        Eigen::Vector2d x0 = compute_mean(src);
        Eigen::Vector2d y0 = compute_mean(target);
        Eigen::Matrix2d H = compute_covariance(src, target, x0, y0);
        
        Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix2d U = svd.matrixU();
        Eigen::Matrix2d V = svd.matrixV();

        Eigen::Matrix2d R = U * V.transpose();
        Eigen::Vector2d t = y0 - R * x0;

        std::for_each(src.begin(), src.end(), [R, t](Eigen::Vector2d &point) { point = R * point + t;});
        
    }

  viewCloud(src, target);
}