#include <iostream>
#include "dataloader.hpp"
#include "viewer.hpp"

int main() {
    std::string data_root = "data/";
    std::cout<<"Loading data"<< std::endl;
    dataset::LaserScanDataset laser_data(data_root);

    dataset::LaserScanDataset::PointCloud src = laser_data[1];
    dataset::LaserScanDataset::PointCloud target = laser_data[2];
    std::cout<< "Length of src: " << src.size() << std::endl;
    std::cout<< "Length of target: " << target.size() << std::endl;
    
    // Sample the source point cloud
    dataset::LaserScanDataset::PointCloud sampled_src;
    sampled_src = sample_points(src, 300);
    std::cout<< "Length of sampled_src: " << sampled_src.size() << std::endl;

    // Find nearest neighbors using Annoy
    dataset::LaserScanDataset::PointCloud correspondences = find_nearest_neighbours(sampled_src, target);
    std::cout<< "Length of correspondences: " << correspondences.size() << std::endl;

    return 0;
}

