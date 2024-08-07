#include <iostream>
#include "dataloader.hpp"
#include "viewer.hpp"
#include "viewer.cpp"
#include "registration.hpp"
#include "registration.cpp"
#include <chrono> // To monitor time
#include <iomanip>

int main()
{
    // Load the dataset
    std::string data_root = "data/";
    dataset::LaserScanDataset laser_data(data_root);

    // Print some basic information about the dataset
    int num_scans = laser_data.size();
    std::cout << "Data Loaded \n--------------------" << std::endl;
    std::cout<<"Available processing time: "<<(num_scans*0.1)<<" seconds"<<std::endl; // 10 scans per second (10 Hz)
    std::cout << "Total number of scans: " << num_scans << std::endl;

    // Number of scans to process
    int scans = num_scans - 1;
    // int scans = 2000;        // Number of Scans to process

    // hyperparameters (tuned for the dataset)
    double pixel_size = 0.08;

    std::cout << "Registering scans \n--------------------" << std::endl;
    std::cout << "Progress \n";

    // Get the starting timepoint
    auto start = std::chrono::high_resolution_clock::now(); // Time point before the execution of the code
    dataset::LaserScanDataset::PointCloud src = laser_data[0];
    dataset::LaserScanDataset::PointCloud target;

    int progress = 0;
    for (int iter = 1; iter <= scans; iter++)
    {
        target = laser_data[iter];
        icp_unknown_correspondences(src, target, pixel_size);
        src = concat_pointclouds(src, target);
        src = downsample(src, 0.13, 1);   
        target.clear();

        progress = 100 * (iter) / scans;
        std::cout << "\r " << progress << " %" << std::flush;
    }

    std::cout << "Processing finished \n--------------------" << std::endl;
    auto end = std::chrono::high_resolution_clock::now(); // Time point after the execution of the code
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end - start);
    std::cout << "\nExecution time: " << duration.count() << " seconds" << std::endl; // Time taken for the execution of the code
    std::cout << "Viewing the point cloud" << std::endl;
    viewCloud(src);

    return 0;
}
