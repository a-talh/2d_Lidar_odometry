#include "viewer.hpp"
#include "../dataloader/dataloader.hpp"
#include <algorithm>
#include <iostream>
#include <open3d/Open3D.h>
#include <open3d/core/ShapeUtil.h>
#include <utility>
#include <vector>
#include "../annoy/src/annoylib.h"      // for annoy::AnnoyIndex
#include "../annoy/src/kissrandom.h"
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

void viewCloud(std::vector<Eigen::Vector2d> &first, const std::vector<Eigen::Vector2d> &second)
{
    first.reserve(first.size() + second.size());
    first.insert(first.end(), second.begin(), second.end());
    viewCloud(first);

}


Eigen::Matrix3d icp_known_corres(std::vector<Eigen::Vector2d> &src, const std::vector<Eigen::Vector2d> &target)
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

Eigen::Matrix3d icp_unknown_correspondence(std::vector<Eigen::Vector2d> &src, const std::vector<Eigen::Vector2d> &target){

    int max_iterations = 50;
    int iter = 0;
    double max_err = 0.1;
    double old_err =  INFINITY;
    double best_err = INFINITY;

    // Sample point cloud
    // std::vector<Eigen::Vector2d> src;
    // std::vector<Eigen::Vector2d> target;
    // std::cout<<"Iteration: "<<max_iterations<<std::endl;
    // src = sample_points(src_org, src_org.size()/2);
    // target = sample_points(target_org, target_org.size()/2);

    // std::vector<Eigen::Vector3d> sampled_src;
    // sampled_src.reserve(src.size()/20);
    std::vector<Eigen::Vector2d> correspondences;
    correspondences.reserve(src.size());
    Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d t = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d t_b = Eigen::Matrix3d::Zero();

    // stuff related to Annoy
    std::vector<int> indices;
    int dimension = 2;
    int n_points = target.size();
    int n_trees = 10;

    // Build the Annoy index for the target point cloud
    Annoy::AnnoyIndex<int, double, Annoy::Euclidean, Annoy::Kiss64Random,Annoy::AnnoyIndexSingleThreadedBuildPolicy> annoy_index(dimension);
    for (int i = 0; i < n_points; i++) {annoy_index.add_item(i, target[i].data());}
    annoy_index.build(n_trees, -1);

        while(true){

            // Find nearest neighbors using Annoy
            std::for_each(src.begin(), src.end(), [&](auto &point) { annoy_index.get_nns_by_vector(point.data(), 1, -1, &indices, nullptr); });
            std::for_each(indices.begin(), indices.end(), [&](int idx) { correspondences.push_back(target[idx]); });

            // Perform ICP with known correspondences
            Eigen::Matrix3d t = icp_known_corres(src, target);

            // Save the transformation
            T = t * T;

            // src = apply_transformation(t, src);

            // Compute the error
            double err = INFINITY;
            err = error(src, target, t);

            // std::cout << "Error: " << err << std::endl;

            // if (err < best_err) {
            //     best_err = err;
            //     t_b = T;
            // }
            // std::cout << "Transformation: \n" << T << std::endl;
            if (err <= max_err || iter == max_iterations) {
                // if (iter == max_iterations) {
                    // std::cout << "Max iterations reached " << std::endl;
                    // T = t_b;
                // }
                // if (err == old_err) {std::cout << "Converged " << std::endl;}
                // if (err <= max_err) {std::cout << "Error threshold reached "<< std::endl;}
                // std::cout << "Final error: " << err << std::endl;
                // std::cout <<"Final Transformation: \n"<<T<<std::endl;
                return T;
            }
            
            old_err = err;
            iter++;
            correspondences.clear();
            indices.clear();
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

std::vector<Eigen::Vector2d> sample_points(const std::vector<Eigen::Vector2d> &vec, int num_samples) {
    std::vector<Eigen::Vector2d> sampled_vec;
    std::sample(vec.begin(),vec.end(),std::back_inserter(sampled_vec) ,num_samples, std::mt19937 {std::random_device{}()});
    return sampled_vec;
}


std::vector<Eigen::Vector2d> concat_pointclouds(std::vector<Eigen::Vector2d> &first, const std::vector<Eigen::Vector2d> &second)
{
    std::vector<Eigen::Vector2d> result = first;
    result.insert(result.end(), second.begin(), second.end());
    return result;
}
