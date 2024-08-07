#include "registration.hpp"
#include "../dataloader/dataloader.hpp"
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

    Eigen::Matrix2d compute_covariance(const std::vector<Eigen::Vector2d> &src, const std::vector<Eigen::Vector2d> &target, const Eigen::Vector2d x0, const Eigen::Vector2d y0)
    {
        Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();

        for (size_t i = 0; i < src.size(); i++)
        {
            cov += (target[i] - y0) * (src[i] - x0).transpose();
        };

        return cov;
    }

    std::vector<Eigen::Vector2d> apply_transformation(const Eigen::Matrix3d &transformation, std::vector<Eigen::Vector2d> &vec)
    {
        Eigen::Matrix2d R = transformation.block<2, 2>(0, 0);
        Eigen::Vector2d t = transformation.block<2, 1>(0, 2);

        std::transform(vec.cbegin(), vec.cend(), vec.begin(), [&R, &t](const auto &p)
                       { return R * p + t; });
        return vec;
    }

    double error(const std::vector<Eigen::Vector2d> &src, const std::vector<Eigen::Vector2d> &target)
    {
        double error = 0.0;

        for (size_t i = 0; i < src.size(); i++)
        {
            error += (target[i] - (src[i])).squaredNorm();
        }

        return error;
    }

    std::unordered_map<Pixel, std::vector<Eigen::Vector2d>> grid_map(const std::vector<Eigen::Vector2d> &vec, const double &pixel_size)
    {
        std::unordered_map<Pixel, std::vector<Eigen::Vector2d>> grid;
        for (const Eigen::Vector2d &point : vec)
        {
            const Pixel p(point, pixel_size);
            grid[p].push_back(point);
        }
        return grid;
    }

    std::vector<Pixel> neighbour_pixels(const Pixel &p, const int pixels = 1)
    {
        std::vector<Pixel> neighbour_pixels;
        neighbour_pixels.reserve(9);
        for (int x = p.i - pixels; x < p.i + pixels + 1; x++)
            for (int y = p.j - pixels; y < p.j + pixels + 1; y++)
                neighbour_pixels.emplace_back(x, y);
        return neighbour_pixels;
    }

    std::vector<Eigen::Vector2d> pixel_points(std::vector<Pixel> pixels, const std::unordered_map<Pixel, std::vector<Eigen::Vector2d>> &target_grid)
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

    std::tuple<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>>
    nearest_neighbours(const std::vector<Eigen::Vector2d> &src, const std::unordered_map<Pixel, std::vector<Eigen::Vector2d>> &target_grid, const double &pixel_size)
    {
        std::vector<Eigen::Vector2d> s_correspondences = {};
        std::vector<Eigen::Vector2d> t_correspondences = {};
        s_correspondences.reserve(src.size());
        t_correspondences.reserve(src.size());
        for (const Eigen::Vector2d &point : src)
        {
            const Pixel p(point, pixel_size);
            const std::vector<Pixel> &neighbour_px = neighbour_pixels(p, 1);
            const std::vector<Eigen::Vector2d> &points = pixel_points(neighbour_px, target_grid);

            if (points.empty())
            {
                continue;
            }

            double min_dist = INFINITY;
            Eigen::Vector2d nearest_neighbour = Eigen::Vector2d::Zero();
            double dist = INFINITY;
            Eigen::Vector2d query_point = Eigen::Vector2d::Zero();
            for (const Eigen::Vector2d &potential_point : points)
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

    Eigen::Matrix3d icp_known_correspondence(std::vector<Eigen::Vector2d> src, const std::vector<Eigen::Vector2d> target)
    {
        const Eigen::Vector2d &x0 = compute_mean(src);
        const Eigen::Vector2d &y0 = compute_mean(target);
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

} // namespace

void icp_unknown_correspondences(std::vector<Eigen::Vector2d> &src, const std::vector<Eigen::Vector2d> &target, const double &pixel_size)
{
    int max_iterations = 16;
    int iter = 0;
    double old_err = INFINITY;

    Eigen::Matrix3d t = Eigen::Matrix3d::Identity();

    std::unordered_map<Pixel, std::vector<Eigen::Vector2d>> target_grid = grid_map(target, pixel_size);
    while (true)
    {
        iter++;
        // Find nearest neighbors
        const std::tuple<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>> &nn = nearest_neighbours(src, target_grid, pixel_size);
        const std::vector<Eigen::Vector2d> &s_correspondences = std::get<0>(nn);
        const std::vector<Eigen::Vector2d> &t_correspondences = std::get<1>(nn);

        // Perform ICP with known correspondences
        t = icp_known_correspondence(s_correspondences, t_correspondences);

        // Apply the transformation
        src = apply_transformation(t, src);

        // Compute the error
        const double &err = error(src, target);

        if (iter == max_iterations || err == old_err)
        {
            return void();
        }

        old_err = err;
    }
}

std::vector<Eigen::Vector2d> concat_pointclouds(std::vector<Eigen::Vector2d> &l_vec, const std::vector<Eigen::Vector2d> &r_vec)
{
    l_vec.reserve(l_vec.size() + r_vec.size());
    l_vec.insert(l_vec.end(), r_vec.begin(), r_vec.end());
    return l_vec;
}

std::vector<Eigen::Vector2d> downsample(const std::vector<Eigen::Vector2d> &vec, const double &pixel_size, const int &n_points)
{
    std::vector<Eigen::Vector2d> filtered_points;
    filtered_points.reserve(vec.size());
    std::unordered_map<Pixel, int> grid2D;
    for (const Eigen::Vector2d &point : vec)
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
    filtered_points.shrink_to_fit();
    return filtered_points;
}
