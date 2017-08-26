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

#ifndef G2O_SE3_RANGE
#define G2O_SE3_RANGE

#include <Eigen/Geometry>
#include <iostream>
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_multi_edge.h"
#include "g2o/stuff/misc.h"
#include "g2o/stuff/macros.h"
#include "g2o/types/slam3d/se3quat.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o_types_api.h"

namespace g2o
{
    class G2O_TYPES_API EdgeSE3Range : public BaseBinaryEdge<1, double, VertexSE3, VertexSE3>
    {
    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeSE3Range();

        virtual bool read(std::istream& is);

        virtual bool write(std::ostream& os) const;

        void computeError();

        virtual void setMeasurement(const double& m)
        {
            _measurement = m;
        }

        virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& , OptimizableGraph::Vertex* )
        {
            return 1.;
        }

        void setVertexOffset(int,  Eigen::Isometry3d&);

        virtual void initialEstimate(const OptimizableGraph::VertexSet& from_, OptimizableGraph::Vertex* to_);

        std::vector<Eigen::Isometry3d> offset =  std::vector<Eigen::Isometry3d>(2, Eigen::Isometry3d::Identity());
    };
}

#endif
