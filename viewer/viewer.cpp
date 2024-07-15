#include "viewer.hpp"
#include "../dataloader/dataloader.hpp"
#include <algorithm>
#include <iostream>
#include <open3d/Open3D.h>
#include <open3d/core/ShapeUtil.h>
#include <utility>
#include <vector>
#include <algorithm> // for std::ranges::sample
#include <random>    // for std::mt19937

namespace
{
    Eigen::Vector2d compute_mean(const std::vector<Eigen::Vector2d> &vec)
    {
        Eigen::Vector2d zero = Eigen::Vector2d::Zero();
        Eigen::Vector2d mean = std::accumulate(vec.begin(), vec.end(), zero );
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

    Eigen::Matrix3d save_transformation(const Eigen::Matrix3d &old_transformation, const Eigen::Matrix3d &transformation)
    {
        // Eigen::Matrix2d R = old_transformation.block<2, 2>(0, 0);
        // Eigen::Matrix2d new_R = transformation.block<2, 2>(0, 0);
        // Eigen::Vector2d t = old_transformation.block<2, 1>(0, 2);
        // Eigen::Vector2d new_t = transformation.block<2, 1>(0, 2);
        // Eigen::Vector2d sum_t = t + new_t;
        // Eigen::Matrix2d sum_R = new_R * R;
        // Eigen::Matrix3d T;
        // T = 
        // T.block<2, 2>(0, 0) = sum_R;
        // T.block<2, 1>(0, 2) = sum_t;

        return transformation * old_transformation;;
    }

    double error(const std::vector<Eigen::Vector2d> &src, const std::vector<Eigen::Vector2d> &target, const Eigen::Matrix3d &transformation)
    {
        // std::vector<Eigen::Vector2d> transformed_points;
        double error = 0.0;
        Eigen::Matrix2d R = transformation.block<2, 2>(0, 0);
        Eigen::Vector2d t = transformation.block<2, 1>(0, 2);

        for (size_t i = 0; i < src.size(); i++)
        {
            error += (target[i] - (R * src[i] + t)).squaredNorm();
        }
        // for (size_t i = 0; i < src.size(); i++)
        // {
        //     error += (target[i] - src[i]).squaredNorm();
        // }
        return error;
    }

    std::vector<Pixel> neighbour_pixels(const Pixel &p, const int pixels = 1)
    {
        std::vector<Pixel> neighbour_pixels;
        neighbour_pixels.reserve(9);
        for (int x = p.i - pixels; x < p.i + pixels+1; x++)
            for (int y = p.j - pixels; y < p.j + pixels+1; y++)
                neighbour_pixels.emplace_back(x, y);
        return neighbour_pixels;
    }

    std::vector<Pixel> GetAdjacentPixels(const Pixel &p, int adjacent_voxels = 1) {
    std::vector<Pixel> pixel_neighborhood;
    for (int i = p.i - adjacent_voxels; i < p.i + adjacent_voxels + 1 ; ++i) {
        for (int j = p.j - adjacent_voxels; j < p.j + adjacent_voxels + 1 ; ++j) {
                pixel_neighborhood.emplace_back(i, j);
        }
    }
    return pixel_neighborhood;
}

    std::vector<Eigen::Vector2d> pixel_points(std::vector<Pixel> &pixels, const std::unordered_map<Pixel, std::vector<Eigen::Vector2d>> &target_grid)
    {
        std::vector<Eigen::Vector2d> points = {};

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

void viewCloud(const std::vector<Eigen::Vector2d> &pcd)
{
    std::vector<Eigen::Vector3d> pts(pcd.size());
    std::transform(pcd.cbegin(), pcd.cend(), pts.begin(), [](const auto &p)
                   { return Eigen::Vector3d(p.x(), p.y(), 0.0); });
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
    T(2, 2) = 1;

    return T;
}

std::unordered_map<Pixel, std::vector<Eigen::Vector2d>> grid_map(const std::vector<Eigen::Vector2d> &vec, double pixel_size)
{   
    std::unordered_map<Pixel, std::vector<Eigen::Vector2d>> grid;

    for (const auto &point : vec)
    {
        const Pixel p(point, pixel_size);
        grid[p].push_back(point);
    }
    
    return grid;
}

std::tuple<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>>
nearest_neighbours(const std::vector<Eigen::Vector2d> &src, const std::unordered_map<Pixel, std::vector<Eigen::Vector2d>> &target_grid, const double &pixel_size)
{
    std::vector<Eigen::Vector2d> s_correspondences = {};
    std::vector<Eigen::Vector2d> t_correspondences = {};
    s_correspondences.reserve(src.size());
    t_correspondences.reserve(src.size());
    for (const auto &point : src)
    {
        const Pixel p(point, pixel_size);
        std::vector<Pixel> neighbour_px = neighbour_pixels(p,1);
        std::vector<Eigen::Vector2d> points = pixel_points(neighbour_px, target_grid);

        if (points.empty())
        {
            continue;
        }

        double min_dist = INFINITY;
        Eigen::Vector2d nearest_neighbour = Eigen::Vector2d::Zero();
        double dist = INFINITY;
        Eigen::Vector2d query_point = Eigen::Vector2d::Zero();
        for (const auto &potential_point : points)
        {
            // Find the nearest neighbour
            dist = (point - potential_point).norm();
            if (dist < min_dist)
            {
                min_dist = dist;
                nearest_neighbour = potential_point;
                query_point = point;
            }
        }

        s_correspondences.emplace_back(query_point);
        t_correspondences.emplace_back(nearest_neighbour);
    }
    s_correspondences.shrink_to_fit();
    t_correspondences.shrink_to_fit();
    return std::make_tuple(s_correspondences, t_correspondences);
}

// std::vector<Eigen::Matrix3d> icp_unknown_correspondence(const std::vector<Eigen::Vector2d> &src_, const std::vector<Eigen::Vector2d> &target, const double &pixel_size)
Eigen::Matrix3d icp_unknown_correspondence(const std::vector<Eigen::Vector2d> &src_, const std::vector<Eigen::Vector2d> &target, const double &pixel_size)
{

    int max_iterations = 50;
    int iter = 0;
    double old_err = INFINITY;

    std::vector<Eigen::Vector2d> src = src_;
    // std::vector<Eigen::Matrix3d> T;
    Eigen::Matrix3d T = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d t = Eigen::Matrix3d::Identity();
    std::vector<Eigen::Vector2d> s_correspondences;
    std::vector<Eigen::Vector2d> t_correspondences;

    std::unordered_map<Pixel, std::vector<Eigen::Vector2d>> target_grid = grid_map(target, pixel_size);
    while (true)
    {
        iter++;
        // Find nearest neighbors
        std::tuple<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>> nn = nearest_neighbours(src, target_grid, pixel_size);
        // std::tuple<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>> nn = findNearestNeighbours(src, target_grid, pixel_size);
        s_correspondences = std::get<0>(nn);
        t_correspondences = std::get<1>(nn);

        // Perform ICP with known correspondences
        t = icp_known_correspondence(s_correspondences, t_correspondences);
        
        // Save the transformation
        // T.emplace_back(t);
        if (iter == 1)
        {
            T = t;
        }
        else {
            T = save_transformation(T, t);
        }
        // T = save_transformation(T, t);

        // Apply the transformation
        src = apply_transformation(t, src);

        // Compute the error
        double err = INFINITY;
        err = error(src, target, t);
        // std::cout<<"Error: "<<err<<std::endl;

        if (iter == max_iterations || err == old_err)
        {
            return T;
        }

        old_err = err;
        s_correspondences.clear();
        t_correspondences.clear();
    }
}

std::vector<Eigen::Vector2d> apply_transformation(const Eigen::Matrix3d &transformation, const std::vector<Eigen::Vector2d> &src)
{
    std::vector<Eigen::Vector2d> transformed_points;
    // transformed_points.reserve(src.size());
    Eigen::Matrix2d R = transformation.block<2, 2>(0, 0);
    Eigen::Vector2d t = transformation.block<2, 1>(0, 2);

    // std::transform(src.begin(), src.end(), std::back_inserter(transformed_points), [&](Eigen::Vector2d &point) {
    //     return R * point + t;
    // });
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

std::vector<Eigen::Vector2d> downsample(const std::vector<Eigen::Vector2d> &vec, const double &pixel_size, const int &n_points)
{
    std::vector<Eigen::Vector2d> filtered_points;
    filtered_points.reserve(vec.size());
    std::unordered_map<Pixel, int> grid2D;
    for (const auto &point : vec)
    {
        const Pixel p(point, pixel_size);
        const auto found = grid2D.find(p);
        if (found == grid2D.end())
        {
            grid2D[p] = 1;
            filtered_points.emplace_back(point);
        }
        else if (grid2D[p] < n_points)
        {
            grid2D[p]++;
            filtered_points.emplace_back(point);
        }
    }

    return filtered_points;
}

std::vector<Eigen::Vector2d> downSampleMean(const std::vector<Eigen::Vector2d> &vec, const double &pixel_size)
{
    std::vector<Eigen::Vector2d> filtered_points;
    filtered_points.reserve(vec.size());
    std::unordered_map<Pixel, std::vector<Eigen::Vector2d>> map2d;
    map2d = grid_map(vec, pixel_size);

    for (const auto &point : vec)
    {
        const Pixel p(point, pixel_size);
        const auto it = map2d.find(p);
        if (it != map2d.end())
        {
            Eigen::Vector2d mean = compute_mean(it->second);
            filtered_points.emplace_back(mean);
            map2d.erase(it);
        }
    }

    return filtered_points;
}

std::tuple<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>> findNearestNeighbours(const std::vector<Eigen::Vector2d> &src, const std::unordered_map<Pixel, std::vector<Eigen::Vector2d>> &target_grid, const double &pixel_size)
{
    std::vector<Eigen::Vector2d> ss;
    std::vector<Eigen::Vector2d> nn;
    for (auto &point : src)
    {

        Pixel p(point, pixel_size);
        std::cout<<"______________________"<<std::endl;
        std::cout<<"Pixel: "<<p.i<<" "<<p.j<<std::endl;
        // std::vector<Pixel> n_px = neighbour_pixels(p, 1);
        std::vector<Pixel> n_px = GetAdjacentPixels(p, 1);
        
        std::cout<<"Neighbour pixels: "<<n_px.size()<<std::endl;
        
        Eigen::Vector2d qp;
        Eigen::Vector2d nn_point;
        double d = std::numeric_limits<double>::max();
        double min_dist = std::numeric_limits<double>::max();
        for (auto &px : n_px)
        {   
            std::cout<<"Pixel: "<<px.i<<" "<<px.j<<std::endl;
            const auto it = target_grid.find(px);
            if (it != target_grid.end())
            {
                std::vector<Eigen::Vector2d> p_vec = it->second;

                for (const auto &pn : p_vec)
                {   
                    std::cout<<"Point: "<<pn.x()<<" "<<pn.y()<<std::endl;
                    d = (point - pn).norm();
                    if (d < min_dist)
                    {
                        min_dist = d;
                        qp = point;
                        nn_point = pn;
                    }
                }
            }
            if (it == target_grid.end())
            {
                min_dist = INFINITY;
            }
        }
        if (min_dist != INFINITY)
        {
            ss.push_back(qp);
            nn.push_back(nn_point);
        }
    }
    return std::make_tuple(ss, nn);
}