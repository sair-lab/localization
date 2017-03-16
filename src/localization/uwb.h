#ifndef UWB_H
#define UWB_H
#include <iostream>
#include <sstream>
#include <string.h>
#include <fstream>
#include <random>
#include <chrono>
#include <math.h>
#include <time.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <boost/concept_check.hpp>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/edge_se3_prior.h>
#include <g2o/types/slam3d/se3quat.h>
#include "types_edge_se3range.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <uwb_driver/UwbRange.h>
#include "lib.h"

using namespace std;

typedef g2o::BlockSolver_6_3 SE3BlockSolver;

typedef g2o::LinearSolverCholmod<SE3BlockSolver::PoseMatrixType> Solver;

typedef Eigen::Matrix<double, 6, 6, Eigen::ColMajor> Matrix6d;

class UWB
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    UWB(ros::NodeHandle); //Takes node as parameter for parameter loading purpose

    void addRangeEdge(const uwb_driver::UwbRange::ConstPtr&);

    void addPriorEdge(int);     //callback function to add edge when a msg is received

//Following function is a fixed version with requirement of total variable number of anchor uwb, taking realPos vector as parameter
    double add_noise(double, double, double);

    void sim_edge_map(int, std::vector<double>); //Generate the Range Map structure 

    double get_map_value(int,int);

    void publish();

    void solve();

    void save();

private:

    ros::Publisher sim_range_pub;

    Jeffsan::CPPTimer timer; //necessary

// for uwb devices    
    int uwb_id;

    int accumulator;

    double distance;

    double distance_err;

// for g2o solver

    Solver *solver;

    SE3BlockSolver *se3blockSolver;
    
    g2o::OptimizationAlgorithmLevenberg *optimizationsolver;
    
    g2o::SparseOptimizer optimizer;

    int iteration_max;

// For UWB anchor parameters reading
    int UWB_count;

    std::vector<int> nodesId;

    std::vector<double> nodesPos;

    std::vector<double> nodesRealPos;

    std::vector<bool> fixed; //manually set value =1 is fixed, true value, =0 is untrustworthy
    
    bool subfixed[3]; //setting fixed-sized excerpt vector

//for Range calculation
    std::map<std::pair<int,int>, double> anchorMap; //the 2D map structure to store the rangecalculations
    
// for anchors
    vector<g2o::VertexSE3*> vertices; //sensor type-> vertices

// for data convertion
    inline g2o::EdgeSE3Prior* create_prior_edge(g2o::VertexSE3*); 
  
    inline g2o::EdgeSE3Range* create_range_edge(g2o::VertexSE3*, g2o::VertexSE3*, double, double);

};

#endif