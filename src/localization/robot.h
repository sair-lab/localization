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

#ifndef ROBOT_H
#define ROBOT_H
#include <iostream>
#include <sstream>
#include <string.h>
#include <fstream>
#include <math.h>
#include <time.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <boost/concept_check.hpp>
#include <nav_msgs/Odometry.h>
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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <nav_msgs/Path.h>

using namespace std;

class Robot
{
public:

    Robot(int ID, bool FLAG_STATIC, int trajectory_length, g2o::SparseOptimizer& optimizer)
        :ID(ID), FLAG_STATIC(FLAG_STATIC), trajectory_length(trajectory_length)
    {
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose(0,3) = 0; 
        pose(1,3) = 0; 
        pose(2,3) = 0.87;
        init(optimizer, pose);
    }; 
    // only call this constructor without following an init()

    Robot(int ID, bool FLAG_STATIC, int trajectory_length)
        :ID(ID), FLAG_STATIC(FLAG_STATIC), trajectory_length(trajectory_length){};
    // call this constructor, then init(optimizer, vertex_init)

    void init(g2o::SparseOptimizer&, Eigen::Isometry3d vertex_init=Eigen::Isometry3d::Identity());

    bool is_static(){return FLAG_STATIC;};

    bool not_static(){return ~FLAG_STATIC;};

    g2o::VertexSE3* new_vertex(unsigned char, std_msgs::Header, g2o::SparseOptimizer&);

    g2o::VertexSE3* last_vertex(unsigned char);

    g2o::VertexSE3* last_vertex();

    std_msgs::Header last_header(unsigned char);

    std_msgs::Header last_header();

    void append_last_header(string);

    nav_msgs::Path* vertices2path();

    geometry_msgs::PoseStamped current_pose();

private:

    map<unsigned char, std_msgs::Header> headers;

    std::vector<std_msgs::Header> header; // headr corresponding to vertices

    vector<g2o::VertexSE3*> vertices; //sensor type-> vertices

    nav_msgs::Path* path;

    map<unsigned char, size_t> type_index; //sensor type -> current vertex index

    size_t index; // current vertex index

    int ID; // Robot ID

    bool FLAG_STATIC;

    int trajectory_length;
};

#endif
