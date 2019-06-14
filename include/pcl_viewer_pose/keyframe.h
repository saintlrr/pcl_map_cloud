#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "pcl_viewer_pose/config.h"
#include "pcl_viewer_pose/graph_slam.h"
#include <pcl/point_cloud.h>

namespace pcl_viewer_pose
{

class Keyframe
{
    public:
    template <typename T>
    pcl::PointCloud<T>::Ptr cloud;
    double accum_dist;
    Eigen::Affine3d T;
    Eigen::Affine3d Ts;
    int index;
    g2o::VertexSE3* node;
    
    public:

    Keyframe();
    ~Keyframe();

};

}

#endif