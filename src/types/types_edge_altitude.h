
#ifndef G2O_SE3_altitude
#define G2O_SE3_altitude

#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_multi_edge.h"
#include <Eigen/Geometry>
#include <iostream>
#include "g2o/stuff/misc.h"
#include "g2o/stuff/macros.h" // plus
#include <g2o/types/slam3d/types_slam3d.h>
#include "g2o/types/slam3d/se3quat.h" // plus 
#include "g2o_types_api.h"
#include <Eigen/Dense>



namespace g2o {    

  
using namespace std;


class G2O_TYPES_API zedge : public BaseBinaryEdge<1, double, VertexSE3, VertexSE3>
{

public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW

     zedge();

     virtual bool read(std::istream& is);

     virtual bool write(std::ostream& os) const;

    void computeError()
    {
      const VertexSE3* v1 = dynamic_cast<const VertexSE3*>(_vertices[0]);

      const VertexSE3* v2 = dynamic_cast<const VertexSE3*>(_vertices[1]);    

      double delta = v1->estimate().translation()[2] + v2->estimate().translation()[2];

      
      if (delta > 0)

      {

          delta = 10000;

      }

      
      if (delta <= 0)

      {

          delta = 0;

      }
     
       cout<<"delta"<< delta <<endl;

      _error[0] = fabs(delta);



     }

      virtual void setMeasurement(const double& m)

      { 
        _measurement = m;
      }


    virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& , OptimizableGraph::Vertex* ) 
    { 
      return 1.;
    }

     virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);
 

 };

 }

#endif