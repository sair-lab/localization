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

void Robot::init(g2o::SparseOptimizer& optimizer)
{
    trajectory_length = 3;

    headers = vector<std_msgs::Header>(trajectory_length, std_msgs::Header());

    for (size_t i = 0; i < trajectory_length; ++i)
    {
        g2o::VertexSE3* vertex = new g2o::VertexSE3();

        vertex->setId(ID + i*10);

        vertex->setEstimate(Eigen::Isometry3d::Identity());

        vertices.push_back(vertex);

        optimizer.addVertex(vertex);
    }

    index = 0;

    // vertices and headers needs to be initilize
}


g2o::VertexSE3* Robot::new_vertex(unsigned char type, std_msgs::Header header, g2o::SparseOptimizer& optimizer)
{
    type_index.insert({type,0});

    if(FLAG_STATIC)
    {
        return last_vertex(type);
    }
    else
    {   
        index = (index+1)%trajectory_length;
        optimizer.removeVertex(vertices[index]);

        auto vertex = new g2o::VertexSE3();
        vertex->setId(index*10 + ID);
        vertex->setEstimate(vertices[index]->estimate());
        vertices[index] = vertex;
        
        type_index[type] = index;
        headers[index] = header;

        return vertex;
    }
}

g2o::VertexSE3* Robot::last_vertex(unsigned char type)
{
    type_index.insert({type,0});

    return vertices[type_index[type]];
}