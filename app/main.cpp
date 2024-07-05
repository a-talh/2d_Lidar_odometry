#include <iostream>
#include "dataloader.hpp"
#include "viewer.hpp"

int main() {
    std::string data_root = "data/";
    std::cout<<"Loading data"<< std::endl;
    dataset::LaserScanDataset laser_data(data_root);

    int num_scans = laser_data.size();
    std::cout<<"Available time "<<(num_scans*0.1)/60<<" minutes"<<std::endl;

    // int iters = num_scans - 1;
    int iters = 50;
    Eigen::Matrix3d T;
    dataset::LaserScanDataset::PointCloud src;
    dataset::LaserScanDataset::PointCloud target;
    dataset::LaserScanDataset::PointCloud transformed_pc;
    dataset::LaserScanDataset::PointCloud result;

    std::cout<<"Applying ICP "<<std::endl;
    for (int i = 0; i < iters; i++)
    {
        src = laser_data[i];
        target = laser_data[i+1];
        std::cout<<i<<std::endl;
        T = icp_unknown_correspondence(src, target);
        laser_data.SetTransformation(T);

    }

    std::cout<<"\nApplying transformation\n"<<std::endl;
    for (int i = 0; i < iters; i++)
    {   
        std::cout<<i<<std::endl;
        if (i == 0)
        {
            transformed_pc = laser_data[i];
            T = laser_data.GetTransformation(i);
            result = apply_transformation(T, transformed_pc);
            target = laser_data[i+1];
            result = concat_pointclouds(result, target);
            laser_data.SetRegisteredPointCloud(result);
            continue;
        }

        transformed_pc = laser_data.GetRegisteredPointCloud();
        T = laser_data.GetTransformation(i);
        result = apply_transformation(T, transformed_pc);
        target = laser_data[i+1];
        result = concat_pointclouds(result, target);
        laser_data.SetRegisteredPointCloud(result);
    }


    // Visualize the point clouds
    result = laser_data.GetRegisteredPointCloud();
    std::cout<<"Number of points in the registered point cloud: "<<result.size()<<std::endl;
    viewCloud(result);
    return 0;
}


