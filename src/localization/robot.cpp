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

#include "robot.h"

void Robot::init(g2o::SparseOptimizer& optimizer, Eigen::Isometry3d vertex_init)
{
    index = 0;
    path = new nav_msgs::Path();

    path->poses = vector<geometry_msgs::PoseStamped>(trajectory_length, geometry_msgs::PoseStamped());
    header = std::vector<std_msgs::Header>(trajectory_length, std_msgs::Header());

    for (int i = 0; i < trajectory_length; ++i)
    {
        g2o::VertexSE3* vertex = new g2o::VertexSE3();

        vertex->setId(ID + i*300);

        vertex->setEstimate(vertex_init);

        vertices.push_back(vertex);

        geometry_msgs::PoseStamped pose;

        if(FLAG_STATIC)
            vertex->setFixed(true);

        optimizer.addVertex(vertex);
    }

    header[0].frame_id = "none";
}


nav_msgs::Path* Robot::vertices2path()
{
    for (int i = 0; i < trajectory_length; ++i)
    {
        int idx = (index+1+i)%trajectory_length;
        tf::poseEigenToMsg(vertices[idx]->estimate(), path->poses[i].pose);
        path->poses[i].header = header[idx];
    }
    path->header = last_header();

    return path;
}


g2o::VertexSE3* Robot::new_vertex(unsigned char type, std_msgs::Header new_header, g2o::SparseOptimizer& optimizer)
{
    type_index.emplace(type, index);
    headers.emplace(type, new_header);

    if(FLAG_STATIC)
    {
        header[index] = new_header;
        return last_vertex(type);
    }
    
    else
    {   
        auto vertex = new g2o::VertexSE3();

        vertex->setEstimate(vertices[index]->estimate());

        index = (index+1)%trajectory_length;

        vertex->setId(index*300 + ID);

        optimizer.removeVertex(vertices[index], false);

        vertices[index] = vertex;

        header[index] = new_header;

        type_index.at(type) = index;

        headers.at(type) = new_header;

        optimizer.addVertex(vertex);

        return vertex;
    }
}


g2o::VertexSE3* Robot::last_vertex(unsigned char type)
{
    type_index.emplace(type, index);
    headers.emplace(type, header[index]);
    return vertices.at(type_index[type]);
}


g2o::VertexSE3* Robot::last_vertex()
{
    return vertices.at(index);
}


std_msgs::Header Robot::last_header(unsigned char type)
{
    headers.emplace(type, header[index]);
    return headers.at(type);
}


std_msgs::Header Robot::last_header()
{
    return header[index];
}


void Robot::append_last_header(string frame_id)
{
    header[index].frame_id += ("-"+frame_id);
}


geometry_msgs::PoseStamped Robot::current_pose()
{
    auto vertex = last_vertex()->estimate();

    geometry_msgs::PoseStamped pose;

    pose.header = last_header();

    tf::poseEigenToMsg(vertex, pose.pose);

    return pose;
}
