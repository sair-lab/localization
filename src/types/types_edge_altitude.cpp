#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"
#include "types_edge_altitude.h"


namespace g2o 


{

  using namespace std;

  using namespace Eigen;

  G2O_REGISTER_TYPE(EDGE_altitude, zedge); // EDGE_RANGE guding

  zedge::zedge():BaseBinaryEdge<1, double, VertexSE3, VertexSE3>(){}  // again anouncement


  bool zedge::read(std::istream& is)
  {
    double meas;
    is >> meas;
    setMeasurement(meas);
    information().setIdentity();
    is >> information()(0,0);
    return true;
  }

  bool zedge::write(std::ostream& os) const
  {
    os  << measurement() << " " << information()(0,0);
    return os.good();
  }



  void zedge::initialEstimate(const OptimizableGraph::VertexSet& from_, OptimizableGraph::Vertex* /*to_*/)
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



  }