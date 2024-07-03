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

    double error(const std::vector<Eigen::Vector2d> &src, const std::vector<Eigen::Vector2d> &target, const Eigen::Matrix2d &R, const Eigen::Vector2d &t)
    {
        double error = 0.0;
        // std::for_each(src.begin(), src.end(), [R, t](Eigen::Vector2d &point) { point = R * point + t; });
        // error = (src - target).norm();
        for (size_t i = 0; i < src.size(); i++)
        {
            error += (R * src[i] + t - target[i]).norm();
        }
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


std::pair<Eigen::Matrix2d, Eigen::Vector2d> icp_known_correspondence(std::vector<Eigen::Vector2d> &src, const std::vector<Eigen::Vector2d> &target)
{   
    Eigen::Vector2d x0 = Eigen::Vector2d::Zero();
    Eigen::Vector2d y0 = Eigen::Vector2d::Zero();
    Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
    Eigen::Vector2d t = Eigen::Vector2d::Zero();
    Eigen::Matrix2d R_ = Eigen::Matrix2d::Identity();
    Eigen::Vector2d t_ = Eigen::Vector2d::Zero();
    int count = 1;
    while (true)
     {
        Eigen::Vector2d x0 = compute_mean(src);
        Eigen::Vector2d y0 = compute_mean(target);
        Eigen::Matrix2d H = compute_covariance(src, target, x0, y0);
        
        Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix2d U = svd.matrixU();
        Eigen::Matrix2d V = svd.matrixV();

        Eigen::Matrix2d R = U * V.transpose();
        Eigen::Vector2d t = y0 - R * x0;

        R_ = R * R_;
        t_ = t_ + t;

        count++;

        if (error(src, target, R_, t_) <  70 || count == 2)
        {   
            // std::cout<<"Count: "<<count<<std::endl;
            break;
        }
    }
    return std::make_pair(R_, t_);
}

// std::pair<Eigen::Matrix2d, Eigen::Vector2d> icp_unknown_correspondence(std::vector<Eigen::Vector2d> &src, const std::vector<Eigen::Vector2d> &target){
// int max_iterations = 1000;
// double max_err = 1e-5;


// // neighbour = NearestNeighbors(n_neighbors=1).fit(Line1.T)
// //     for i in range(MaxIter):

// //         # find correspondences of points
// //         QInd = np.array([i[0] for i in neighbour.kneighbors(Line2.T, return_distance=False)]) 
// //         PInd = np.arange(Line2.shape[1])

// //         # update Line2 and error
// //         # Now that you know the correspondences, use your implementation
// //         # of icp with known correspondences and perform an update
// //         EOld = E
// //         [Line2, E] = icp_known_corresp(Line1, Line2, QInd, PInd)

// //         print('Error value on ' + str(i) + ' iteration is: ', E)

// //         # TODO: perform the check if we need to stop iterating
// //         if E - EOld == 0 or E < Epsilon:
// //             break
        
// //     # return the final error 
// //     return E

// }


std::vector<Eigen::Vector2d> apply_transformation(const std::pair <Eigen::Matrix2d, Eigen::Vector2d> &transformation, const std::vector<Eigen::Vector2d> &src){
    std::vector<Eigen::Vector2d> transformed(src.size());
    // std::for_each(src.begin(), src.end(), [transformation, transformed](Eigen::Vector2d &point) { point = transformation.first * point + transformation.second; });
    transformed = src;
    return transformed;
}

std::vector<Eigen::Vector2d> sample_points(const std::vector<Eigen::Vector2d> &vec, int num_samples) {
    std::vector<Eigen::Vector2d> sampled_vec;
    std::sample(vec.begin(),vec.end(),std::back_inserter(sampled_vec) ,num_samples, std::mt19937 {std::random_device{}()});
    return sampled_vec;
}

std::vector<Eigen::Vector2d> find_nearest_neighbours(const std::vector<Eigen::Vector2d> &src, const std::vector<Eigen::Vector2d> &target) {
    std::vector<Eigen::Vector2d> nearest_neighbours;
    int dimension = 2;
    int n_points = target.size();
    int n_trees = 10;
    
    Annoy::AnnoyIndex<int, double, Annoy::Euclidean, Annoy::Kiss64Random,Annoy::AnnoyIndexSingleThreadedBuildPolicy> annoy_index(dimension);

    for (int i = 0; i < n_points; i++) {annoy_index.add_item(i, target[i].data());}

    annoy_index.build(n_trees, -1);

    std::vector<int> indices;
    for (const auto &src_vec : src) { annoy_index.get_nns_by_vector(src_vec.data(), 1, -1, &indices, nullptr); }

    for (int idx : indices) { nearest_neighbours.push_back(target[idx]); }
    
    return nearest_neighbours;
}