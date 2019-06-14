// #include <iostream>
// #include <dirent.h>
// #include <sys/stat.h>
// #include <unistd.h>
// #include <fstream>
// #include <string>
// #include <vector>
// #include <math.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl/visualization/pcl_visualizer.h>
#include "pcl_viewer_pose/csv_visualization.h"

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    std::string points_file_name = "redo.csv";
    std::string line;
    ifstream points_file (points_file_name);
    size_t i = 0;
    if (points_file.is_open())
    {
        while ( getline (points_file,line) )
        {
            cloud->width    = i+1;
            cloud->height   = 1;
            cloud->is_dense = true;
            cloud->points.resize (cloud->width * cloud->height);
            std::string  points_ele_string;
            std::istringstream tmp(line);
            std::vector<double> point_vec;
            while( getline(tmp, points_ele_string, ','))
            {
                double point_ele = std::stold(points_ele_string);
                point_vec.push_back(point_ele);
            }
            cloud->points[i].x = point_vec[0];
            cloud->points[i].y = point_vec[1];
            cloud->points[i].z = point_vec[2];
            i++;
        }
        points_file.close();
    }

    // Visualization
    pcl::visualization::PCLVisualizer viewer ("pose pointcloud");
    viewer.addCoordinateSystem (1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler (cloud, 230, 230, 230);
    viewer.addPointCloud (cloud, cloud_color_handler, "csvcloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.01, "csvcloud");

    // pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to test_pcd.pcd." << std::endl;

    // for (size_t i = 0; i < cloud->points.size(); ++i)
    //     std::cerr << "    " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << std::endl;

    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
                viewer.spinOnce ();
            }
    return 1;
}