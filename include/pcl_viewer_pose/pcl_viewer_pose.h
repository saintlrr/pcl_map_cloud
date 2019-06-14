#ifndef PCL_VIEWER_POSE_H
#define PCL_VIEWER_POSE_H

#include "pcl_viewer_pose/config.h"
#include "pcl_viewer_pose/graph_slam.h"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace pcl_viewer_pose
{
    // globale variables
    int NUM, DOWNSAMPLE_RATE, ITER_NUM, BEGIN_FRAME, END_FRAME;
    bool MATCH_FLAG, COLOR_FLAG;
    std::string NAME_FLAG, FILTER_TYPE, OUTLIER_FILTER_TYPE, SCAN_TYPE;
    double NDT_EPSILON, NDT_STEPSIZE, NDT_RESOLUTION, KEYFRAME_DELTA_TRANS,
           KEYFRAME_DELTA_ANGLE;

    void readConfig(const std::string& configFile = "config/config.txt");
    void showHelp(char *program_name);

    void readConfig(const std::string& configFile)
    {
        pcl_viewer_pose::Config::setConfigFile(configFile);

        NUM = Config::get("NUM", 140);
        DOWNSAMPLE_RATE = Config::get("DOWNSAMPLE_RATE", 5);
        ITER_NUM = Config::get("ITER_NUM", 10);
        MATCH_FLAG = Config::get("MATCH_FLAG", true);
        COLOR_FLAG = Config::get("COLOR_FLAG", true);
        NAME_FLAG = Config::get("NAME_FLAG", std::string("seq"));
        FILTER_TYPE = Config::get("FILTER_TYPE", std::string("None"));
        BEGIN_FRAME = Config::get("BEGIN_FRAME", 1);
        END_FRAME = Config::get("END_FRAME", -1);
        OUTLIER_FILTER_TYPE = Config::get("OUTLIER_FILTER_TYPE", std::string("None"));
        SCAN_TYPE = Config::get("SCAN_TYPE", std::string("None"));
        NDT_EPSILON = Config::get("NDT_EPSILON", 0.01);
        NDT_STEPSIZE = Config::get("NDT_STEPSIZ", 0.1);
        NDT_RESOLUTION = Config::get("NDT_RESOLUTION", 1);
        KEYFRAME_DELTA_ANGLE = Config::get("KEYFRAME_DELTA_ANGLE", 2);
        KEYFRAME_DELTA_TRANS = Config::get("KEYFRAME_DELTA_TRANS", 0.5);
    }

    // This function displays the help
    void showHelp(char *program_name)
    {
        std::cout << std::endl;
        std::cout << "Usage: " << program_name << " pcd_files_directory"
                << " pose_file_path" << std::endl;
        std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" 
                << std::endl;
        std::cout << "-h:  Show this help." << std::endl;
    }

    // downsample frame
    template <typename T>
    std::vector<T> downSample(std::vector<T> vec, int rate = DOWNSAMPLE_RATE)
    {
        std::vector<T> vec_new;
        typename std::vector<T>::iterator vec_iterator;
        for (vec_iterator = vec.begin() + BEGIN_FRAME -1; 
            vec_iterator < vec.end() + END_FRAME +1; 
            vec_iterator += DOWNSAMPLE_RATE)
        {
            vec_new.push_back(*vec_iterator);
        }
        return vec_new;
    }

    // filter and downsample point cloud
    template <typename PointT>
    void downSample_filter(
        typename boost::shared_ptr<pcl::PointCloud<PointT>> source_cloud,
        typename boost::shared_ptr<pcl::PointCloud<PointT>> source_cloud_filtered, 
        std::string filter_type = "None")
    {
        clock_t start_time, end_time;
        start_time = clock();
        if (filter_type == "VoxelGrid")
        {
            pcl::VoxelGrid<PointT> sor;
            sor.setInputCloud(source_cloud);
            sor.setLeafSize(0.1f, 0.1f, 0.1f);
            sor.filter(*source_cloud_filtered);
        }
        else if (filter_type == "ApproVoxelGrid")
        {
            pcl::ApproximateVoxelGrid<PointT> avg;
            avg.setLeafSize(0.1f, 0.1f, 0.1f);
            avg.setInputCloud(source_cloud);
            avg.filter(*source_cloud_filtered);
        }
        else if (filter_type == "Progressive")
        {
            pcl::PointIndicesPtr ground(new pcl::PointIndices);
            pcl::ProgressiveMorphologicalFilter<PointT> pmf;
            pmf.setMaxWindowSize(20);
            pmf.setSlope(1.0f);
            pmf.setInitialDistance(0.5f);
            pmf.setInputCloud(source_cloud);
            pmf.setMaxDistance(3.0f);
            pmf.extract(ground->indices);

            pcl::ExtractIndices<PointT> extract;
            extract.setInputCloud(source_cloud);
            extract.setIndices(ground);
            extract.filter(*source_cloud_filtered);
        }
        else if (filter_type == "SACSeg")
        {
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr ground(new pcl::PointIndices);
            // Create the segmentation object
            pcl::SACSegmentation<PointT> seg;
            // Optional
            seg.setOptimizeCoefficients(true);
            // Mandatory
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.01);

            seg.setInputCloud(source_cloud);
            seg.segment(*ground, *coefficients);

            pcl::ExtractIndices<PointT> extract;
            extract.setInputCloud(source_cloud);
            extract.setIndices(ground);
            extract.filter(*source_cloud_filtered);
        }
        else if (filter_type == "Octree")
        {
            pcl::octree::OctreePointCloud<PointT> octree(0.02);
            octree.setInputCloud(source_cloud);
            octree.addPointsFromInputCloud();
            octree.getOccupiedVoxelCenters(source_cloud_filtered->points);
            source_cloud_filtered->width = source_cloud_filtered->size();
            source_cloud_filtered->height = 1;
        }
        else if (filter_type == "None")
        {
            *source_cloud_filtered = *source_cloud;
        }
        end_time = clock();
        std::cout << filter_type << " downsample filter Time : " 
                << (double)(end_time - start_time) / CLOCKS_PER_SEC 
                << "s" << std::endl;
    }

    // outlier remove point cloud
    template <typename PointT>
    void outlier_filter(
        typename boost::shared_ptr<pcl::PointCloud<PointT>> source_cloud,
        typename boost::shared_ptr<pcl::PointCloud<PointT>> source_cloud_filtered, 
        std::string filter_type = "None")
    {
        clock_t start_time, end_time;
        start_time = clock();
        if (filter_type == "Statistical")
        {
            pcl::StatisticalOutlierRemoval<PointT> sor;
            sor.setInputCloud(source_cloud);
            sor.setMeanK(50);
            sor.setStddevMulThresh(1.0);
            sor.filter(*source_cloud_filtered);
        }
        else if (filter_type == "None")
        {
            *source_cloud_filtered = *source_cloud;
        }
        end_time = clock();
        std::cout << filter_type << " outlier filter Time : " 
                << (double)(end_time - start_time) / CLOCKS_PER_SEC 
                << "s" << std::endl;
    }

    // add color in height
    template <typename PointT>
    void addColorByHeight(
        typename boost::shared_ptr<pcl::PointCloud<PointT>> source_cloud,
        typename boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> target_cloud)
    {
        target_cloud->width = source_cloud->width;
        target_cloud->height = source_cloud->height;
        target_cloud->is_dense = source_cloud->is_dense;
        target_cloud->points.resize(target_cloud->width*target_cloud->height);
        PointT min_pt, max_pt;
        const pcl::PointCloud<PointT> cloud = *source_cloud;
        pcl::getMinMax3D<pcl::PointXYZI>(cloud, min_pt, max_pt);
        std::vector<double> zlist;
        for (typename pcl::PointCloud<PointT>::iterator ii = 
                (*source_cloud).begin(); 
            ii<(*source_cloud).end(); ii++)
            {   double min_thresh = min_pt.z, max_thresh =max_pt.z;
                if(ii->z < min_thresh || ii->z > max_thresh)
                    continue;
                int point_index = ii - (*source_cloud).begin();
                target_cloud->points[point_index].x = ii->x;
                target_cloud->points[point_index].y = ii->y;
                target_cloud->points[point_index].z = ii->z;
                // std::cout << "z : " << ii->z << std::endl;
                zlist.push_back(ii->z);
                target_cloud->points[point_index].r = 100;
                target_cloud->points[point_index].g = int(((ii->z - min_thresh)/(
                max_thresh - min_thresh))*255);
                target_cloud->points[point_index].b = int(((ii->z - min_thresh)/(
                max_thresh - min_thresh))*255);
                // std::cout<<"rgb : "<< int(target_cloud->points[point_index].r) << " "
                //          << int(target_cloud->points[point_index].g) << " "
                //          << int(target_cloud->points[point_index].b) << std::endl;
            }
        auto it_max = std::max_element(std::begin(zlist), std::end(zlist));
        auto it_min = std::min_element(std::begin(zlist), std::end(zlist));
        std::cout << "z min :" << *it_min << " index : " << it_min - zlist.begin() 
                << "    z max: " << *it_max <<" index : " << it_max -zlist.begin()
                << " size : " << zlist.size() << std::endl;
        std::cout << "min_pt : " << min_pt.z << " " <<min_pt.x <<" " << min_pt.y 
            << "     max_pt : " << max_pt.z << " " <<max_pt.x << " " << max_pt.y
            << std::endl;
    }

    // Scan Matching Algorithm
    template <typename PointT>
    Eigen::Affine3d
    ScanMatching(typename boost::shared_ptr<pcl::PointCloud<PointT>> source_cloud,
        typename boost::shared_ptr<pcl::PointCloud<PointT>> target_cloud, 
        std::string scan_type = "ICP")
    {   
        clock_t start_time, end_time;
        start_time = clock();
        Eigen::Affine3d transformation = Eigen::Affine3d::Identity();
        if (scan_type == "ICP")
        {
            pcl::IterativeClosestPoint<PointT, PointT> icp;
            icp.setInputSource(source_cloud);
            icp.setInputTarget(target_cloud);
            icp.setMaxCorrespondenceDistance(0.05);
            icp.setMaximumIterations(ITER_NUM);
            icp.setTransformationEpsilon(1e-8);
            icp.setEuclideanFitnessEpsilon(1);
            icp.align(*source_cloud);
            if (icp.hasConverged())
            {
                std::cout << "ICP has converged" << icp.hasConverged()
                        << ", score is " << icp.getFitnessScore()
                        << std::endl;
            }
            Eigen::Matrix4f tmp = icp.getFinalTransformation();
            Eigen::Matrix4d tf = tmp.cast<double>();
            transformation.matrix() = tf;
        } 
        else if (scan_type == "NDT")
        {
            pcl::NormalDistributionsTransform<PointT, PointT> ndt;
            ndt.setTransformationEpsilon (NDT_EPSILON);
            ndt.setStepSize(NDT_STEPSIZE);
            ndt.setResolution(NDT_RESOLUTION);
            ndt.setMaximumIterations(ITER_NUM);
            ndt.setInputSource(source_cloud);
            ndt.setInputTarget(target_cloud);
            std::cout << "start to align" << std::endl;
            ndt.align(*source_cloud);
            std::cout << "end of align" << std::endl;
            if (ndt.hasConverged())
            {
                std::cout << "NDT has converged" << ndt.hasConverged()
                        << ", score is " << ndt.getFitnessScore()
                        << std::endl;
            }
            Eigen::Matrix4f tmp = ndt.getFinalTransformation();
            Eigen::Matrix4d tf = tmp.cast<double>();
            transformation.matrix() = tf;
        }
        end_time = clock();
        std::cout << "translation between before and after scan matching: " 
                << transformation.translation() << std::endl;
        std::cout << scan_type << " scan matching time : " 
                << (double)(end_time - start_time) / CLOCKS_PER_SEC 
                << "s" << std::endl;
        return transformation;   
    }
}
#endif