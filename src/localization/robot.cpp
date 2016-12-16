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
    trajectory_length = 200;
    index = 0;
    velocity = geometry_msgs::TwistWithCovariance();

    path = new nav_msgs::Path();
    path->poses = vector<geometry_msgs::PoseStamped>(trajectory_length, geometry_msgs::PoseStamped());

    for (size_t i = 0; i < trajectory_length; ++i)
    {
        g2o::VertexSE3* vertex = new g2o::VertexSE3();

        vertex->setId(ID + i*10);

        vertex->setEstimate(vertex_init);

        vertices.push_back(vertex);

        geometry_msgs::PoseStamped pose;

        if(FLAG_STATIC)
            vertex->setFixed(true);

        optimizer.addVertex(vertex);
    }

    header.frame_id = "none";
}


nav_msgs::Path* Robot::vertices2path()
{
    for (size_t i= 0; i < trajectory_length; ++i)
        tf::poseEigenToMsg(vertices[(index+1+i)%trajectory_length]->estimate(), path->poses[i].pose);

    return path;
}


g2o::VertexSE3* Robot::new_vertex(unsigned char type, std_msgs::Header new_header, g2o::SparseOptimizer& optimizer)
{
    type_index.emplace(type, index);
    headers.emplace(type, new_header);
    header = new_header;

    if(FLAG_STATIC)
    {
        return last_vertex(type);
    }
    else
    {   
        auto vertex = new g2o::VertexSE3();

        vertex->setEstimate(vertices[index]->estimate());

        index = (index+1)%trajectory_length;

        vertex->setId(index*10 + ID);

        optimizer.removeVertex(vertices[index], false);

        vertices[index] = vertex;        

        type_index.at(type) = index;

        headers.at(type) = new_header;

        optimizer.addVertex(vertex);

        return vertex;
    }
}


g2o::VertexSE3* Robot::last_vertex(unsigned char type)
{
    type_index.emplace(type, index);
    headers.emplace(type, header);
    return vertices.at(type_index[type]);
}


g2o::VertexSE3* Robot::last_vertex()
{
    return vertices.at(index);
}


std_msgs::Header Robot::last_header(unsigned char type)
{
    headers.emplace(type, header);
    return headers.at(type);
}


std_msgs::Header Robot::last_header()
{
    return header;
}
