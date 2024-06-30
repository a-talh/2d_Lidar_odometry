#include <iostream>
#include "dataloader.hpp"
#include "viewer.hpp"

int main() {
    std::string data_root = "data/";
    std::cout<<"Loading data"<< std::endl;
    dataset::LaserScanDataset laser_data(data_root);

    int iters = 15;

    
    std::cout << "Implementing ICP"<< std::endl;
    int last_precent = -5;
    for (int i = 0; i < iters; i++)
    {
    int percent = (i * 100) / iters;
    if (percent % 5 == 0 && percent != last_precent)
    {
        std::cout << percent << "% done" << std::endl;
        last_precent = percent;
    }
    std::vector<Eigen::Vector2d> src = laser_data[i];
    std::vector<Eigen::Vector2d> target = laser_data[i+1];
    std::pair<Eigen::Matrix2d, Eigen::Vector2d> transformation = implement_icp(src, target);
    laser_data.SetTransformation(transformation);
    }
    
    std::cout << "Applying Transformations"<< std::endl;
    last_precent = -5;

    dataset::LaserScanDataset::PointCloud src = laser_data[0];
    dataset::LaserScanDataset::PointCloud target = laser_data[1];
    // std::cout<<"Points in first src scan: "<<src.size()<<std::endl;
    // std::cout<<"Points in first target scan: "<<target.size()<<std::endl;
    std::pair<Eigen::Matrix2d, Eigen::Vector2d> recovered_transformation = laser_data.GetTransformation(0);
    dataset::LaserScanDataset::PointCloud transformed_src = apply_transformation(recovered_transformation, src);
    transformed_src.reserve(transformed_src.size() + target.size());
    transformed_src.insert(transformed_src.end(), target.begin(), target.end());
    // std::cout<<"Points in first transformed scan: "<<transformed_src.size()<<std::endl;
    laser_data.AddRegisteredPointCloud(transformed_src);
    transformed_src.clear();
    for (int i = 1; i < iters; i++)
    {
    int percent = (i * 100) / iters;
    if (percent % 5 == 0 && percent != last_precent)
    {
        std::cout << percent << "% done" << std::endl;
        last_precent = percent;
    }
    // std::cout<<"Iteration : "<<i<<std::endl;
    dataset::LaserScanDataset::PointCloud registered_pcd = laser_data.GetRegisteredPointCloud();
    // std::cout<<"Points in recovered scan: "<<registered_pcd.size()<<std::endl;
    dataset::LaserScanDataset::PointCloud target = laser_data[i+1];
    // std::cout<<"Points in target scan: "<<target.size()<<std::endl;
    std::pair<Eigen::Matrix2d, Eigen::Vector2d> recovered_transformation = laser_data.GetTransformation(i);
    dataset::LaserScanDataset::PointCloud transformed_src = apply_transformation(recovered_transformation, registered_pcd);
    // std::cout<<"Points in transformed scan: "<<transformed_src.size()<<std::endl;
    transformed_src.reserve(transformed_src.size() + target.size());
    transformed_src.insert(transformed_src.end(), target.begin(), target.end());

    // std::cout<<"Points in transformed scan + target scan: "<<transformed_src.size()<<std::endl;
    laser_data.SetRegisteredPointCloud(transformed_src);
    transformed_src.clear();
    }


    dataset::LaserScanDataset::PointCloud recovered_pcd = laser_data.GetRegisteredPointCloud();
    // std::cout<<"Points in combined pcd:" << recovered_pcd.size()<< std:: endl;
    viewCloud(recovered_pcd);
    return 0;
}
