
// #include "g2o/core/base_vertex.h"
// #include "g2o/core/base_binary_edge.h"
// #include "g2o/core/base_multi_edge.h"

// #include <g2o/types/slam3d/types_slam3d.h>
// #include <g2o/types/sba/types_sba.h>

// #include <Eigen/Geometry>
// #include <iostream>


#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"

#include "zedge.h"


namespace g2o 


{

  using namespace std;

  using namespace Eigen;

  G2O_REGISTER_TYPE(EDGE_RANGE, zedge); // EDGE_RANGE guding

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




  G2O_REGISTER_TYPE(EDGE_RANGE1, yedge); // EDGE_RANGE guding

  yedge::yedge():BaseBinaryEdge<1, double, VertexSE3, VertexSE3>(){}  // again anouncement

 

  bool yedge::read(std::istream& is)
  {
    double meas;
    is >> meas;
    setMeasurement(meas);
    information().setIdentity();
    is >> information()(0,0);
    return true;
  }

  bool yedge::write(std::ostream& os) const
  {
    os  << measurement() << " " << information()(0,0);
    return os.good();
  }



void yedge::initialEstimate(const OptimizableGraph::VertexSet& from_, OptimizableGraph::Vertex* /*to_*/)
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




// G2O_REGISTER_TYPE(EDGE_PROJECT_P2MC, ydedge); // EDGE_RANGE guding

//   ydedge::ydedge():BaseBinaryEdge<2, Vector2D, VertexSE3, VertexSE3>(){}  // again anouncement

 

//   bool ydedge::read(std::istream& is)
//   {
//     double meas;
//     is >> meas;
//     setMeasurement(meas);
//     information().setIdentity();
//     is >> information()(0,0);
//     return true;
//   }

//   bool ydedge::write(std::ostream& os) const
//   {
//     os  << measurement() << " " << information()(0,0);
//     return os.good();
//   }



// void ydedge::initialEstimate(const OptimizableGraph::VertexSet& from_, OptimizableGraph::Vertex* /*to_*/)
//     {
//         VertexSE3* v1 = dynamic_cast<VertexSE3*>(_vertices[0]);

//         VertexSE3* v2 = dynamic_cast<VertexSE3*>(_vertices[1]);

//         if (from_.count(v1) == 1)
//         {
//             Eigen::Transform<double, 3, 1> delta = (v1->estimate().inverse()*v2->estimate());

//             double norm =  delta.translation().norm();

//             double alpha = _measurement/norm;

//             delta.translation()=(delta.translation()*alpha);

//             v2->setEstimate(v1->estimate()*delta);
//         }
//         else
//         {
//             Eigen::Transform<double, 3, 1> delta = (v2->estimate().inverse()*v1->estimate());

//             double norm =  delta.translation().norm();

//             double alpha = _measurement/norm;

//             delta.translation()=(delta.translation()*alpha);

//             v1->setEstimate(v2->estimate()*delta);
//         }
//     }





  G2O_REGISTER_TYPE(EDGE_RANGE2, edgedistance); // EDGE_RANGE guding

  edgedistance::edgedistance():BaseBinaryEdge<1, double, VertexSE3, VertexSE3>(){}  // again anouncement


  bool edgedistance::read(std::istream& is)
  {
    double meas;
    is >> meas;
    setMeasurement(meas);
    information().setIdentity();
    is >> information()(0,0);
    return true;
  }

  bool edgedistance::write(std::ostream& os) const
  {
    os  << measurement() << " " << information()(0,0);
    return os.good();
  }



void edgedistance::initialEstimate(const OptimizableGraph::VertexSet& from_, OptimizableGraph::Vertex* /*to_*/)
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


  
G2O_REGISTER_TYPE(EDGE_disz, disz);

disz::disz() : BaseBinaryEdge<2, Vector2D, VertexSE3, VertexSE3>()
{
    information().setIdentity();
  }


  bool disz::read(std::istream& is)
  {
    // measured keypoint
    for (int i=0; i<2; i++)
      is >> _measurement[i];
    setMeasurement(_measurement);
    // information matrix is the identity for features, could be changed to allow arbitrary covariances
    information().setIdentity();
    return true;
  }

  bool disz::write(std::ostream& os) const
  {
    for (int i=0; i<2; i++)
      os  << measurement()[i] << " ";
    return os.good();
  }




void disz::initialEstimate(const OptimizableGraph::VertexSet& from_, OptimizableGraph::Vertex* /*to_*/)
    {
        VertexSE3* v1 = dynamic_cast<VertexSE3*>(_vertices[0]);

        VertexSE3* v2 = dynamic_cast<VertexSE3*>(_vertices[1]);

        if (from_.count(v1) == 1)
        {
            Eigen::Transform<double, 3, 1> delta = (v1->estimate().inverse()*v2->estimate());

            double norm =  delta.translation().norm();

            double alpha = _measurement[0]/norm;

            delta.translation()=(delta.translation()*alpha);

            v2->setEstimate(v1->estimate()*delta);
        }
        else
        {
            Eigen::Transform<double, 3, 1> delta = (v2->estimate().inverse()*v1->estimate());

            double norm =  delta.translation().norm();

            double alpha = _measurement[0]/norm;

            delta.translation()=(delta.translation()*alpha);

            v1->setEstimate(v2->estimate()*delta);
        }
    }






  }