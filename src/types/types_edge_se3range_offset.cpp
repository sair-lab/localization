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

#include "types_edge_se3range_offset.h"
#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"

namespace g2o
{
    using namespace std;

    using namespace Eigen;

    G2O_REGISTER_TYPE(EDGE_RANGE_OFFSET, EdgeSE3RangeOffset);

    EdgeSE3RangeOffset::EdgeSE3RangeOffset():BaseBinaryEdge<1, double, VertexSE3, VertexSE3>()
    {
        _offsetFrom = 0; _offsetTo = 0;

        _cacheFrom = 0; _cacheTo = 0;

        resizeParameters(2);

        installParameter(_offsetFrom, 0);

        installParameter(_offsetTo, 1);
    }  


    bool EdgeSE3RangeOffset::read(std::istream& is)
    {
        int pidFrom, pidTo;

        is >> pidFrom >> pidTo;

        if (! setParameterId(0,pidFrom))
            return false;

        if (! setParameterId(1,pidTo))
            return false;

        double meas;

        is >> meas;

        setMeasurement(meas);

        information().setIdentity();

        is >> information()(0,0);

        return true;
    }


    bool EdgeSE3RangeOffset::write(std::ostream& os) const
    {
        os << parameter(0)->id() << " ";
        
        os << parameter(1)->id() << " ";

        os << measurement() << " " << information()(0,0);

        return os.good();
    }


    void EdgeSE3RangeOffset::initialEstimate(const OptimizableGraph::VertexSet& from_, OptimizableGraph::Vertex* /*to_*/)
    {
        VertexSE3* v1 = dynamic_cast<VertexSE3*>(_vertices[0]);

        VertexSE3* v2 = dynamic_cast<VertexSE3*>(_vertices[1]);

        if (from_.count(v1) == 1)
        {
            Eigen::Transform<double, 3, 1> delta = (v1->estimate().inverse()*v2->estimate());

            double norm =  delta.translation().norm();

            double alpha = _measurement/norm;

            delta.translation()=(delta.translation()*alpha);

            v2->setEstimate(v1->estimate()*delta);
        }
        else
        {
            Eigen::Transform<double, 3, 1> delta = (v2->estimate().inverse()*v1->estimate());

            double norm =  delta.translation().norm();

            double alpha = _measurement/norm;

            delta.translation()=(delta.translation()*alpha);

            v1->setEstimate(v2->estimate()*delta);
        }
    }


    void EdgeSE3RangeOffset::computeError()
    {
        Vector3D dt = _cacheFrom->n2w().translation() - _cacheTo->n2w().translation();

        _error[0] = _measurement - dt.norm();
    }


    bool EdgeSE3RangeOffset::resolveCaches()
    {
        assert(_offsetFrom && _offsetTo);

        ParameterVector pv(1);

        pv[0]= _offsetFrom;

        resolveCache(_cacheFrom, (OptimizableGraph::Vertex*)_vertices[0], "CACHE_SE3_OFFSET", pv);

        pv[0]= _offsetTo;

        resolveCache(_cacheTo, (OptimizableGraph::Vertex*)_vertices[1], "CACHE_SE3_OFFSET", pv);

        return (_cacheFrom && _cacheTo);
    }
}