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

    std::vector<Pixel> neighbour_pixels(const Pixel &p)
    {
        return {Pixel(p.i - 1, p.j - 1), Pixel(p.i - 1, p.j), Pixel(p.i - 1, p.j + 1),
                Pixel(p.i, p.j - 1), p , Pixel(p.i, p.j + 1),
                Pixel(p.i + 1, p.j - 1), Pixel(p.i + 1, p.j), Pixel(p.i + 1, p.j + 1)};
    }

    std::vector<Eigen::Vector2d> pixel_points(std::vector<Pixel> &pixels, const std::unordered_map<Pixel, std::vector<Eigen::Vector2d>> &target_grid)
    {
        std::vector<Eigen::Vector2d> points;
        for (const auto &pixel : pixels)
        {
            const auto it = target_grid.find(pixel);
            if (it != target_grid.end())
            {
                points.insert(points.end(), it->second.begin(), it->second.end());
            }
        }
        return points;
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
}

std::tuple<std::vector<Eigen::Vector2d>,std::vector<Eigen::Vector2d>> 
nearest_neighbours(const std::vector<Eigen::Vector2d> &src, const std::unordered_map<Pixel, std::vector<Eigen::Vector2d>> &target_grid, double pixel_size){
   std::vector<Eigen::Vector2d> s_correspondences(src.size());
   std::vector<Eigen::Vector2d> t_correspondences(src.size());
   for (const auto &point : src)
    {
        const Pixel p(point, pixel_size);
        std::vector<Pixel> neighbour_px = neighbour_pixels(p);
        std::vector<Eigen::Vector2d> neighbours = pixel_points(neighbour_px, target_grid);

        if (neighbours.empty())
        {
            continue;
        }

        double min_dist = INFINITY;
        Eigen::Vector2d nearest_neighbour = Eigen::Vector2d::Zero();
        double dist = 0.0;
        for (const auto &potential_neighbour : neighbours)
        {
            // Find the nearest neighbour
            dist = (point - potential_neighbour).norm();
            if (dist < min_dist)
            {
                min_dist = dist;
                nearest_neighbour = potential_neighbour;
            }
        }
        
        s_correspondences.emplace_back(point);
        t_correspondences.emplace_back(nearest_neighbour);
    }   
    return std::make_tuple(s_correspondences, t_correspondences);
}



// Eigen::Matrix3d icp_unknown_correspondence(const std::vector<Eigen::Vector2d> &src, const std::vector<Eigen::Vector2d> &target){

//     int max_iterations = 5;
//     int iter = 0;
//     double max_err = 0.1;
//     double old_err =  INFINITY;

//     std::vector<Eigen::Vector2d> correspondences;
//     correspondences.reserve(src.size());
//     Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
//     Eigen::Matrix3d t = Eigen::Matrix3d::Zero();
//     // Eigen::Matrix3d t_b = Eigen::Matrix3d::Zero();


//         while(true){

//             // Find nearest neighbors 

//             // Perform ICP with known correspondences
//             Eigen::Matrix3d t = icp_known_correspondence(src, target);

//             // Save the transformation
//             T = t * T;

//             // src = apply_transformation(t, src);

//             // Compute the error
//             double err = INFINITY;
//             err = error(src, target, t);

//             if (err <= max_err || iter == max_iterations) {
//                 return T;
//             }
            
//             old_err = err;
//             iter++;
//             correspondences.clear();
//         }
// }


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
