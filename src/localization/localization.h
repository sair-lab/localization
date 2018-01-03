// Copyright (c) <2016>, <Nanyang Technological University> All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.

// 3. Neither the name of the copyright holder nor the names of its contributors
// may be used to endorse or promote products derived from this software without
// specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef LOCALIZATION_H
#define LOCALIZATION_H

// #define TIME_DOMAIN
//comment above line if you want to use uwb from timedomain

#include <iostream>
#include <sstream>
#include <string.h>
#include <stdio.h>
#include <fstream>
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
#include "types_edge_se3range.h"
#include "types_edge_se3range_offset.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#ifdef TIME_DOMAIN
#include <uwb_driver/UwbRange.h>
#else
#include <bitcraze_lps_estimator/UwbRange.h>
#endif
#include <sensor_msgs/Imu.h>
#include <dynamic_reconfigure/server.h>
#include <localization/localizationConfig.h>
#include <message_filters/subscriber.h>
#include <std_msgs/Float64.h>
#ifdef RELATIVE_LOCALIZATION
#include <uwb_reloc/uwbTalkData.h>
#endif
#include "lib.h"
#include "robot.h"

using namespace std;

typedef g2o::BlockSolver_6_3 SE3BlockSolver;

typedef g2o::LinearSolverCholmod<SE3BlockSolver::PoseMatrixType> Solver;
// typedef g2o::LinearSolverCSparse<SE3BlockSolver::PoseMatrixType> Solver;


int test();

const struct SensorType
{
    unsigned char general = 0;
    unsigned char pose = 1;
    unsigned char range = 2;
    unsigned char twist = 3;
    unsigned char imu = 4;
}sensor_type;

class Localization
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Localization(ros::NodeHandle);

    ~Localization();

    void solve();

    void publish();

#ifdef TIME_DOMAIN
    void addRangeEdge(const uwb_driver::UwbRange::ConstPtr&);
#else
    void addRangeEdge(const bitcraze_lps_estimator::UwbRange::ConstPtr&);
#endif
    void addPoseEdge(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);

    void addLidarEdge(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_cov_);

    void addImuEdge(const sensor_msgs::Imu::ConstPtr&);

    void addTwistEdge(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr&);
#ifdef RELATIVE_LOCALIZATION
    void addRLRangeEdge(const uwb_reloc::uwbTalkData::ConstPtr&);
#endif
    void configCallback(localization::localizationConfig&, uint32_t);

private:

    Jeffsan::CPPTimer timer;

    ros::Publisher pose_realtime_pub;

    ros::Publisher pose_optimized_pub;

    ros::Publisher path_optimized_pub;

// for robots
    std::vector<int> nodesId;

    std::vector<double> nodesPos;

    map<unsigned char, Robot> robots;

    unsigned char self_id;

    double robot_max_velocity;

    g2o::VertexSE3* key_vertex;

    int trajectory_length;

    int number_measurements;

    double distance_outlier;

    double minimum_optimize_error;

// for g2o solver
    Solver *solver;

    SE3BlockSolver *se3blockSolver;

    g2o::OptimizationAlgorithmLevenberg *optimizationsolver;

    g2o::SparseOptimizer optimizer;

    std::vector<Eigen::Isometry3d> offsets = std::vector<Eigen::Isometry3d>(3, Eigen::Isometry3d::Identity());

    int iteration_max;

// for debug
    string realtime_filename, optimized_filename, name_prefix, frame_source, frame_target;

    ofstream file;

    bool flag_save_file, publish_tf, publish_range, publish_pose, publish_twist, publish_lidar, publish_imu, publish_relative_range;

    tf::TransformBroadcaster br;

    tf::Transform transform;

// for data convertion
    inline g2o::EdgeSE3* create_se3_edge_from_twist(g2o::VertexSE3*, g2o::VertexSE3*, geometry_msgs::TwistWithCovariance&, double);

    inline g2o::EdgeSE3Range* create_range_edge(g2o::VertexSE3*, g2o::VertexSE3*, double, double);

    inline geometry_msgs::Twist pose2twist(geometry_msgs::Pose, geometry_msgs::Pose, double);

    inline Eigen::Isometry3d twist2transform(geometry_msgs::TwistWithCovariance&, Eigen::MatrixXd&, double);

    inline void save_file(geometry_msgs::PoseStamped, string);

public:
    void set_file();
    void set_file(std::vector<double> antennaOffset);

};

#endif
