#include <iostream>
#include "dataloader.hpp"
#include "viewer.hpp"

int main() {
    std::string data_root = "data/";
    std::cout<<"Reading data"<< std::endl;
    dataset::LaserScanDataset laser_data(data_root);
    std::cout<<"Size of data :" << laser_data.size()<< std::endl;
    
    std::vector<Eigen::Vector2d> pcd = laser_data[0];
    std::vector<Eigen::Vector2d> pcd2 = laser_data[1];
    
    // for (int i = 0; i < laser_data.size() - 1; i++)
    // {
    //     std::vector<Eigen::Vector2d> pcd = laser_data[i];
    //     std::vector<Eigen::Vector2d> pcd2 = laser_data[i+1];
    // };

    implement_icp(pcd, pcd2);

    return 0;
}