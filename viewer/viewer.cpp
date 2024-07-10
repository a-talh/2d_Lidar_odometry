#include "viewer.hpp"
#include "../dataloader/dataloader.hpp"
#include <algorithm>
#include <iostream>
#include <open3d/Open3D.h>
#include <open3d/core/ShapeUtil.h>
#include <utility>
#include <vector>
#include <algorithm>                    // for std::ranges::sample
#include <random>                       // for std::mt19937

namespace
{
    Eigen::Vector2d compute_mean(const std::vector<Eigen::Vector2d> &vec)
    {
        Eigen::Vector2d zero = Eigen::Vector2d::Zero();
        Eigen::Vector2d mean = std::accumulate(vec.begin(), vec.end(), zero);
        return mean / vec.size();
    }

    Eigen::Matrix2d compute_covariance(const std::vector<Eigen::Vector2d> &src, const std::vector<Eigen::Vector2d> &target, const Eigen::Vector2d &mean1, const Eigen::Vector2d &mean2)
    {
        Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();

        for (size_t i = 0; i < src.size(); i++)
        {
            cov += (target[i] - mean2) * (src[i] - mean1).transpose();
        };

        return cov;
    }

    double error(std::vector<Eigen::Vector2d> &src, const std::vector<Eigen::Vector2d> &target, const Eigen::Matrix3d &transformation)
    {
        // std::vector<Eigen::Vector2d> transformed_points;
        double error = 0.0;
        Eigen::Matrix2d R = transformation.block<2, 2>(0, 0);
        Eigen::Vector2d t = transformation.block<2, 1>(0, 2);

        for (size_t i = 0; i < src.size(); i++)
        {
            src[i] = (R * src[i] + t);
            error  += (target[i] - src[i]).squaredNorm();

        }
        // for (size_t i = 0; i < src.size(); i++)
        // {
        //     error += (target[i] - src[i]).squaredNorm();
        // }
        return error;
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

std::vector<Eigen::Vector3d> get_points(const open3d::geometry::PointCloud &pcd)
{
    return pcd.points_;
}

void viewCloud(const std::vector<Eigen::Vector2d> &first, const std::vector<Eigen::Vector2d> &second)
{   
    std::vector<Eigen::Vector2d> result = first;
    result.reserve(result.size() + second.size());
    result.insert(result.end(), second.begin(), second.end());
    viewCloud(result);

}


Eigen::Matrix3d icp_known_correspondence(std::vector<Eigen::Vector2d> &src, const std::vector<Eigen::Vector2d> &target)
{   
    Eigen::Vector2d x0 = compute_mean(src);
    Eigen::Vector2d y0 = compute_mean(target);
    Eigen::Matrix2d H = compute_covariance(src, target, x0, y0);
    
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d U = svd.matrixU();
    Eigen::Matrix2d V = svd.matrixV();

    Eigen::Matrix2d R = U * V.transpose();
    Eigen::Vector2d t = y0 - R * x0;

    Eigen::Matrix3d T = Eigen::Matrix3d::Zero();
    T.block<2, 2>(0, 0) = R;
    T.block<2, 1>(0, 2) = t;

    return T;
}

std::unordered_map<Pixel, std::vector<Eigen::Vector2d>> grid_map(const std::vector<Eigen::Vector2d> &vec, double pixel_size){
    std::unordered_map<Pixel, std::vector<Eigen::Vector2d>> grid;
    grid.reserve(vec.size());
    for (const auto &point : vec)
    {
        const Pixel p(point, pixel_size);
        grid[p].emplace_back(point);
    }
    return grid;


Eigen::Matrix3d icp_unknown_correspondence(std::vector<Eigen::Vector2d> &src, const std::vector<Eigen::Vector2d> &target){

    int max_iterations = 5;
    int iter = 0;
    double max_err = 0.1;
    double old_err =  INFINITY;

    std::vector<Eigen::Vector2d> correspondences;
    correspondences.reserve(src.size());
    Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d t = Eigen::Matrix3d::Zero();
    // Eigen::Matrix3d t_b = Eigen::Matrix3d::Zero();


        while(true){

            // Find nearest neighbors 

            // Perform ICP with known correspondences
            Eigen::Matrix3d t = icp_known_correspondence(src, target);

            // Save the transformation
            T = t * T;

            // src = apply_transformation(t, src);

            // Compute the error
            double err = INFINITY;
            err = error(src, target, t);

            if (err <= max_err || iter == max_iterations) {
                return T;
            }
            
            old_err = err;
            iter++;
            correspondences.clear();
        }
}


std::vector<Eigen::Vector2d> apply_transformation(const Eigen::Matrix3d &transformation, const std::vector<Eigen::Vector2d> &src){
    std::vector<Eigen::Vector2d> transformed_points;
    Eigen::Matrix2d R = transformation.block<2, 2>(0, 0);
    Eigen::Vector2d t = transformation.block<2, 1>(0, 2);

    for (size_t i = 0; i < src.size(); i++)
    {
        transformed_points.push_back(R * src[i] + t);
    }
    return transformed_points;
}

std::vector<Eigen::Vector2d> concat_pointclouds(std::vector<Eigen::Vector2d> &first, const std::vector<Eigen::Vector2d> &second)
{
    std::vector<Eigen::Vector2d> result = first;
    result.insert(result.end(), second.begin(), second.end());
    return result;
}
