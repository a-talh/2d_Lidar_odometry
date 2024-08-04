#include <iostream>
#include "dataloader.hpp"
#include "viewer.hpp"
#include "viewer.cpp"
#include <ctime>    // For time()
#include <chrono>   // To monitor time
#include <iomanip>

int main()
{
    int choice = 3; // choose the pointcloud to be used 0 = test_pc, 1 = Lidar data, 2 = Lidar data different approach

    if (choice == 0){
        const std::string filename1 = "data/src.ply";
        const std::string filename = "data/target.ply";

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
        std::transform(points.begin(), points.end(), std::back_inserter(src), [](const auto &p)
                    { return Eigen::Vector2d(p.x(), p.y()); });
        std::transform(points1.begin(), points1.end(), std::back_inserter(target), [](const auto &p)
                    { return Eigen::Vector2d(p.x(), p.y()); });

        std::cout << "Read points from both pointclouds successfully!" << std::endl;
        viewCloud(src, target);

        // Apply ICP
        Eigen::Matrix3d T;
        double pixel_size = 2.0;
        T = icp_unknown_correspondence(src, target, pixel_size);
        std::cout << "Applying transformations " << std::endl;
        src = apply_transformation(T, src);
        src = concat_pointclouds(src, target);
        // pixel_size = 0.05;
        // src = downSampleMean(src, pixel_size);
        viewCloud(src, target);
        }
    
    if (choice == 1){
        std::string data_root = "data/";
        std::cout<<"Loading data"<< std::endl;
        dataset::LaserScanDataset laser_data(data_root);

        int num_scans = laser_data.size();
        std::cout<<"Available time "<<(num_scans*0.1)/60<<" minutes"<<std::endl;

        // int iters = num_scans - 1;
        int iters = 30;        // Number of iterations to run ICP
        double pixel_size = 0.08; 

        std::vector<dataset::LaserScanDataset::Transformation> T;
        dataset::LaserScanDataset::PointCloud src;
        dataset::LaserScanDataset::PointCloud target;
        int j = 0;
 
        std::cout<<"Applying ICP \n============== "<<std::endl;
        std::cout<<"Progress \n";

        for (int i = 0; i < iters; i++)
        {
            src = laser_data[i];
            target = laser_data[i+1];
            T.emplace_back(icp_unknown_correspondence(src, target, pixel_size));
            j = 100 * (i+1) / iters;
            std::cout<<"\r "<<j<<" %"<<std::flush;
        }
        j = 0;
        std::cout<<"Applying Transformations \n============== "<<std::endl;
        std::cout<<"Progress \n";
        pixel_size = 0.1;
        for (int i = 0; i < iters; i++)
        {   if (i == 0){
            src = apply_transformation(T[i], laser_data[i]);
            src = concat_pointclouds(src, laser_data[i+1]);
            src = downsample(src, pixel_size, 1);
            }
            if (i > 0){
            src = laser_data.GetRegisteredPointCloud();
            src = apply_transformation(T[i], src);
            src = concat_pointclouds(src, laser_data[i+1]);
            src = downSampleMean(src, pixel_size);
            laser_data.SetRegisteredPointCloud(src);
            src.clear();
            }  
            j = 100 * (i+1) / iters;
            std::cout<<"\r "<<j<<" %"<<std::flush;
        }
        src = laser_data.GetRegisteredPointCloud();
        std::cout<<"\nNumber of points in the registered point cloud: "<<src.size()<<std::endl;
        viewCloud(src);
    }

    if (choice == 2){
        std::string data_root = "data/";
        dataset::LaserScanDataset laser_data(data_root);

        int num_scans = laser_data.size();
        // std::cout<<"Data loaded, available processing time "<<(num_scans*0.1)/60<<" minutes"<<std::endl;
        std::cout<<"total scans: "<<num_scans<<std::endl;
        int iters = num_scans - 1;
        // int iters = 2000;        // Number of Scans to process
        double pixel_size = 0.08; 

        dataset::LaserScanDataset::Transformation T;
        dataset::LaserScanDataset::PointCloud src;
        dataset::LaserScanDataset::PointCloud target;
        int j = 0;
 
        std::cout<<"Applying ICP & Transformations \n________________________ "<<std::endl;
        std::cout<<"Progress \n";
        // Get the starting timepoint
        auto start = std::chrono::high_resolution_clock::now();     // Time point before the execution of the code
        src = laser_data[0];
        for (int i = 1; i <= iters; i++)
        {   
            target = laser_data[i];
            T = icp_unknown_correspondence(src, target, pixel_size);
            src = apply_transformation(T, src);
            src = concat_pointclouds(src, target);
            // src = downSampleMean(src, 0.1);
            src = downsample(src, 0.1, 1);
            target.clear();
            
            j = 100 * (i) / iters;
            std::cout<<"\r "<<j<<" %"<<std::flush;
        }
        auto end = std::chrono::high_resolution_clock::now();       // Time point after the execution of the code
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(end - start);
        std::cout << "Execution time: " << duration.count() << " seconds" << std::endl; // Time taken for the execution of the code
        std::cout<<"\nNumber of points in the registered point cloud: "<<src.size()<<std::endl;
        viewCloud(src);
    }

    if (choice == 3){
        std::string data_root = "data/";
        dataset::LaserScanDataset laser_data(data_root);

        int num_scans = laser_data.size();
        // std::cout<<"Data loaded, available processing time "<<(num_scans*0.1)/60<<" minutes"<<std::endl;
        std::cout<<"total scans: "<<num_scans<<std::endl;
        int iters = num_scans - 1;
        // int iters = 2000;        // Number of Scans to process
        double pixel_size = 0.08; 

        dataset::LaserScanDataset::Transformation T;
        dataset::LaserScanDataset::PointCloud src;
        dataset::LaserScanDataset::PointCloud target;
        int j = 0;
 
        std::cout<<"Applying ICP & Transformations \n________________________ "<<std::endl;
        std::cout<<"Progress \n";
        // Get the starting timepoint
        auto start = std::chrono::high_resolution_clock::now();     // Time point before the execution of the code
        src = laser_data[0];
        for (int i = 1; i <= iters; i++)
        {   
            target = laser_data[i];
            icp_unknown_correspondences(src, target, pixel_size);
            src = concat_pointclouds(src, target);
            src = downsample(src, 0.1, 1);
            target.clear();
            
            j = 100 * (i) / iters;
            std::cout<<"\r "<<j<<" %"<<std::flush;
        }
        auto end = std::chrono::high_resolution_clock::now();       // Time point after the execution of the code
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(end - start);
        std::cout << "Execution time: " << duration.count() << " seconds" << std::endl; // Time taken for the execution of the code
        std::cout<<"\nNumber of points in the registered point cloud: "<<src.size()<<std::endl;
        viewCloud(src);
    }

    else if (choice != 0 && choice != 1 && choice != 2 && choice != 3){
        std::cout<<"Invalid choice"<<std::endl;
    }
    
    return 0;
}
