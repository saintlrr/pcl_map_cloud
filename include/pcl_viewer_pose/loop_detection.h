#ifndef LOOP_DETECTION_H
#define LOOP_DETECTION_H

#include "pcl_viewer_pose/keyframe.h"
#include "pcl_viewer_pose/config.h"
#include "pcl_viewer_pose/graph_slam.h"
#include "pcl_viewer_pose/pcl_viewer_pose.h"

namespace pcl_viewer_pose
{
    bool loop_detection(std::vector<Keyframe> keyframe_list, 
                        Keyframe keyframe,
                        GraphSLAM& graph_slam)
    {
        double loop_min_length_thresh = Config::get("LOOP_MIN_LENGTH_THRESH", 50);
        double loop_r_thresh = Config::get("LOOP_R_thresh", 1);
        double score_thresh = Config::get("FIT_SCORE", 0.5);
        
        for(Keyframe k = keyframe_list.begin(); 
            k < keyframe_list.end(); k++)
        {
            Eigen::Affine3d T_delta = Eigen::Affine3d::Identity();
            T_delta = k.T.inverse() * keyframe.T;
            double dx = T_delta.translation().norm();
            if (dx > loop_r_thresh)
                continue;
            auto source_cloud = keyframe.cloud;
            auto target_cloud = k.cloud;
            pcl::transformPointCloud(*source_cloud, *source_cloud, keyframe.Ts);
            pcl::transformPointCloud(*target_cloud, *target_cloud, k.Ts);
            pcl::Registration<PointT, PointT>::Ptr reg = Matching(
                source_cloud, target_cloud, SCAN_TYPE);
            Eigen::Affine3d transformation = Eigen::Affine3d::Identity();
            Eigen::Matrix4f tmp = reg->getFinalTransformation();
            Eigen::Matrix4d tf = tmp.cast<double>();
            transformation.matrix() = tf;
            double score = reg->getFitnessScore();
            if (score > score_thresh)
                continue;            

            Eigen::Affine3d T_delta_m = k.Ts.inverse()*transformation*keyframe.Ts;
            Eigen::MatrixXd information_matrix = cal_information_matrix(
                keyframe.cloud, k.cloud, T_delta_m);
            auto edge = graph_slam->add_se3_edge(
                                    k.node,
                                    keyframe.node,
                                    T_delta_m,
                                    information_matrix);
            graph_slam->add_robust_kernel(edge, "NONE", 1.0);
            std::cout<<"Loop found! "<< std::endl;
        }
    }

    // Scan Matching Algorithm
    template <typename PointT>
    pcl::Registration<PointT, PointT>::Ptr
    Matching(typename boost::shared_ptr<pcl::PointCloud<PointT>> source_cloud,
        typename boost::shared_ptr<pcl::PointCloud<PointT>> target_cloud, 
        std::string scan_type = "ICP")
    {   
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
            return *icp;
            
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
            return *ndt;
        }   
    }
}

#endif