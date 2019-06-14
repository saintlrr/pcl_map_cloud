#ifndef GRAPH_SLAM_HPP
#define GRAPH_SLAM_HPP

#include <memory>

#include <boost/format.hpp>
#include <pcl/point_cloud.h>
#include <g2o/stuff/macros.h>
#include <g2o/core/factory.h>
#include <g2o/core/eigen_types.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>
// #include <g2o/types/slam3d/edge_se3_xyzprior.h>
#include <g2o/types/slam3d/edge_se3_prior.h>
// #include <g2o/edge_se3_plane.hpp>
// #include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
// #include <g2o/edge_se3_priorvec.hpp>
// #include <g2o/edge_se3_priorquat.hpp>
#include <g2o/core/sparse_optimizer.h>

#include "pcl_viewer_pose/config.h"

// namespace g2o {
//   class VertexSE3;
//   class VertexPlane;
//   class VertexPointXYZ;
//   class EdgeSE3;
//   class EdgeSE3Plane;
//   class EdgeSE3PointXYZ;
//   class EdgeSE3PriorXY;
//   class EdgeSE3PriorXYZ;
//   class EdgeSE3PriorVec;
//   class EdgeSE3PriorQuat;
//   class RobustKernelFactory;
// }

namespace pcl_viewer_pose
{

class GraphSLAM
{
public:
    GraphSLAM(const std::string &solver_type = "lm_var");
    ~GraphSLAM();

    /**
   * @brief add a SE3 node to the graph
   * @param pose
   * @return registered node
   */
    g2o::VertexSE3 *add_se3_node(const Eigen::Isometry3d &pose);

    /**
   * @brief add a plane node to the graph
   * @param plane_coeffs
   * @return registered node
   */
    g2o::VertexPlane *add_plane_node(const Eigen::Vector4d &plane_coeffs);

    /**
   * @brief add a point_xyz node to the graph
   * @param xyz
   * @return registered node
   */
    g2o::VertexPointXYZ *add_point_xyz_node(const Eigen::Vector3d &xyz);

    /**
   * @brief add an edge between SE3 nodes
   * @param v1  node1
   * @param v2  node2
   * @param relative_pose  relative pose between node1 and node2
   * @param information_matrix  information matrix (it must be 6x6)
   * @return registered edge
   */
    g2o::EdgeSE3 *add_se3_edge(g2o::VertexSE3 *v1, g2o::VertexSE3 *v2, const Eigen::Isometry3d &relative_pose, const Eigen::MatrixXd &information_matrix);

    /**
   * @brief add an edge between an SE3 node and a plane node
   * @param v_se3    SE3 node
   * @param v_plane  plane node
   * @param plane_coeffs  plane coefficients w.r.t. v_se3
   * @param information_matrix  information matrix (it must be 3x3)
   * @return registered edge
   */
    //   g2o::EdgeSE3Plane* add_se3_plane_edge(g2o::VertexSE3* v_se3, g2o::VertexPlane* v_plane, const Eigen::Vector4d& plane_coeffs, const Eigen::MatrixXd& information_matrix);

    /**
   * @brief add an edge between an SE3 node and a point_xyz node
   * @param v_se3        SE3 node
   * @param v_xyz        point_xyz node
   * @param xyz          xyz coordinate
   * @param information  information_matrix (it must be 3x3)
   * @return registered edge
   */
    g2o::EdgeSE3PointXYZ *add_se3_point_xyz_edge(g2o::VertexSE3 *v_se3, g2o::VertexPointXYZ *v_xyz, const Eigen::Vector3d &xyz, const Eigen::MatrixXd &information_matrix);

    /**
   * @brief add a prior edge to an SE3 node
   * @param v_se3
   * @param xy
   * @param information_matrix
   * @return
   */

    g2o::EdgeSE3PriorXYZ *add_se3_prior_xyz_edge(g2o::VertexSE3 *v_se3, const Eigen::Vector3d &xyz, const Eigen::MatrixXd &information_matrix);
    // g2o::EdgeSE3Prior* add_se3_prior_edge(g2o::VertexSE3* v_se3, const Eigen::Vector3d& xyz, const Eigen::MatrixXd& information_matrix);

    void add_robust_kernel(g2o::OptimizableGraph::Edge *edge, const std::string &kernel_type, double kernel_size);

    /**
   * @brief perform graph optimization
   */
    void optimize(int num_iterations);

    /**
   * @brief save the pose graph
   * @param filename  output filename
   */
    void save(const std::string &filename);

    static double cal_weight(const double &a, 
                             const double &max_x, 
                             const double &min, 
                             const double &max, 
                             const double &x)
    {
        double w = (1.0 - std::exp(-a*x)) / (1.0 - std::exp(-a*max_x));
        return min*min +(max*max-min*min)*w; 
    }

    template <typename PointT>
    static Eigen::MatrixXd cal_information_matrix(
                        const pcl::PointCloud<PointT>::ConstPtr &source_cloud,
                        const pcl::PointCloud<PointT>::ConstPtr &target_cloud,
                        const Eigen::Affine3d relpose);

public:
    g2o::RobustKernelFactory *robust_kernel_factory;
    std::unique_ptr<g2o::SparseOptimizer> graph; // g2o graph
};

template <typename PointT>
Eigen::MatrixXd GraphSLAM::cal_information_matrix(
                        const pcl::PointCloud<PointT>::ConstPtr &source_cloud,
                        const pcl::PointCloud<PointT>::ConstPtr &target_cloud,
                        const Eigen::Affine3d relpose)
{
    // read config file
    double fit_score_thresh = Config::get("FIT_SCORE_THRESH", 0.5);
    double var_gain_a = Config::get("VAR_GAIN_A", 10);
    double min_std_x = Config::get("MIN_STD_X", 0.1);
    double max_std_x = Config::get("MAX_STD_X", 2);
    double min_std_q = Config::get("MIN_STD_Q", 0.05);
    double max_std_q = Config::get("MAX_STD_Q", 0.2);

    double fit_score = GraphSLAM::cal_fit_score(
                        source_cloud, target_cloud, relpose);
    w_x = GraphSLAM::cal_weight(var_gain_a, fit_score_threh, min_std_x,
                                max_std_x, fit_score);
    w_q = GraphSLAM::cal_weight(var_gain_a, fit_score_threh, min_std_q,
                                max_std_q, fit_score);

    
}

} // namespace pcl_viewer_pose

#endif // GRAPH_SLAM_HPP