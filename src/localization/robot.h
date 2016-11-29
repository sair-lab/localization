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
#include <g2o/types/slam3d/types_slam3d.h>
#include "types_edge_se3range.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using namespace std;

class Robot
{
public:

    Robot(int id):ID(id){};

    int next_pose_id();


private:

    vector<std_msgs::Header> headers;

    vector<g2o::VertexSE3> poses;

    vector<g2o::EdgeSE3Range*> sensor_range;

    vector<g2o::EdgeSE3*> sensor_pose;

    vector<int> poses_global_id; // pose glbal id in a team robots

    int poses_number; //length of poses

    int ID; // Robot ID

    bool FLAG_STATIC;
};

#endif
