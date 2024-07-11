#include <iostream>
#include "dataloader.hpp"
#include "viewer.hpp"
#include <ctime>
#include <iomanip>

int main() {

    const std::string filename = "data/src.ply";
    const std::string filename1 = "data/target.ply";

    open3d::geometry::PointCloud pcd;
    open3d::geometry::PointCloud pcd1;

    open3d::io::ReadPointCloud(filename, pcd);
    open3d::io::ReadPointCloud(filename1, pcd1);

    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3d> points1;
    // get points from pointclouds
    points = get_points(pcd);
    points1 = get_points(pcd1);

    std::vector<Eigen::Vector2d> src;
    std::vector<Eigen::Vector2d> target;
    // convert 3D points to 2D points
    std::transform(points.begin(), points.end(), std::back_inserter(src), [](const auto &p) {
        return Eigen::Vector2d(p.x(), p.y());
    });
    std::transform(points1.begin(), points1.end(), std::back_inserter(target), [](const auto &p) {
        return Eigen::Vector2d(p.x(), p.y());
    });

    std::cout << "Read points from both pointclouds successfully!" << std::endl;
    viewCloud(src, target);
    Eigen::Matrix3d T;
    std::vector<Eigen::Vector2d> result;
    double pixel_size = 1;
    int points_per_grid = 1;
    src = downsample(src, pixel_size, points_per_grid);
    T = icp_unknown_correspondence(src, target, pixel_size);
    result = apply_transformation(T, src);
    viewCloud(result, target);
    
    // std::string data_root = "data/";
    // std::cout<<"Loading data"<< std::endl;
    // dataset::LaserScanDataset laser_data(data_root);

    // int num_scans = laser_data.size();
    // std::cout<<"Available time "<<(num_scans*0.1)/60<<" minutes"<<std::endl;

    // int iters = num_scans - 1;
    // int iters = 1;
    // dataset::LaserScanDataset::Transformation T;  
    // dataset::LaserScanDataset::PointCloud src;
    // dataset::LaserScanDataset::PointCloud target;
    // dataset::LaserScanDataset::PointCloud transformed_pc;
    // dataset::LaserScanDataset::PointCloud result;

    // std::cout<<"Applying ICP "<<std::endl;
    // for (int i = 0; i < iters; i++)
    // {
    //     src = laser_data[i];
    //     target = laser_data[i+1];
    //     std::cout<<i<<std::endl;
    //     T = icp_unknown_correspondence(src, target);
    //     laser_data.SetTransformation(T);

    // }

    // std::cout<<"\nApplying transformation\n"<<std::endl;
    // for (int i = 0; i < iters; i++)
    // {   
    //     std::cout<<i<<std::endl;
    //     if (i == 0)
    //     {
    //         transformed_pc = laser_data[i];
    //         T = laser_data.GetTransformation(i);
    //         result = apply_transformation(T, transformed_pc);
    //         // result = sample_points(result, result.size()/2);
    //         target = laser_data[i+1];
    //         result = concat_pointclouds(result, target);
    //         laser_data.SetRegisteredPointCloud(result);
    //         continue;
    //     }

    //     transformed_pc = laser_data.GetRegisteredPointCloud();
    //     T = laser_data.GetTransformation(i);
    //     result = apply_transformation(T, transformed_pc);
    //     // result = sample_points(result, result.size()/2);
    //     target = laser_data[i+1];
    //     result = concat_pointclouds(result, target);
    //     laser_data.SetRegisteredPointCloud(result);
    // }

    
    // // Visualize the point clouds
    // result = laser_data.GetRegisteredPointCloud();
    // std::cout<<"Number of points in the registered point cloud: "<<result.size()<<std::endl;
    // viewCloud(result);
    return 0;
}


