#ifndef G2O_SBA_TYPES
#define G2O_SBA_TYPES

#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_multi_edge.h"
#include <Eigen/Geometry>
#include <iostream>

#include "g2o/stuff/misc.h"
#include "g2o/stuff/macros.h" // plus

#include <g2o/types/slam3d/types_slam3d.h>
#include "g2o/types/slam3d/se3quat.h" // plus 

#include "g2o_types_api1.h"
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

      // const VertexSBAPointXYZ* v2 = dynamic_cast<const VertexSBAPointXYZ*>(_vertices[1]);
    

      double delta = v1->estimate().translation()[2] - v2->estimate().translation()[2];

      // double h1 = v1->estimate().translation()[0];
      // double h2 = v1->estimate().translation()[2];
      // double h3 = v1->estimate().translation()[1];

      

      // double h=0;
      // if (h1>=0) 

      //    h=h;
    
      // if (h2<=0)

      //    h=h;

      // if (h3>=0)

      //    h=h;


      //  if (h1<0)

      //    h=h+5;

      //  if (h2>0)

      //    h=h+5;

      //  if (h3<0)

      //    h=h+5;

      // else if (h1<0||h2>0||h3<0)

      //    h=100;

     
       // cout<<"h"<< 100 <<endl;

      _error[0] = fabs(_measurement - delta);



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






class G2O_TYPES_API yedge : public BaseBinaryEdge<1, double, VertexSE3, VertexSE3>
{

public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW

     yedge();

     virtual bool read(std::istream& is);

     virtual bool write(std::ostream& os) const;

    void computeError()
    {
      const VertexSE3* v1 = dynamic_cast<const VertexSE3*>(_vertices[0]);

      const VertexSE3* v2 = dynamic_cast<const VertexSE3*>(_vertices[1]);

      // const VertexSBAPointXYZ* v2 = dynamic_cast<const VertexSBAPointXYZ*>(_vertices[1]);
    

      double delta = v1->estimate().translation()[1] - v2->estimate().translation()[1];

      _error[0] = fabs(_measurement - delta);

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






class G2O_TYPES_API edgedistance : public BaseBinaryEdge<1, double, VertexSE3, VertexSE3>
{

public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW

     edgedistance();

     virtual bool read(std::istream& is);

     virtual bool write(std::ostream& os) const;

    void computeError()
    {
      const VertexSE3* v1 = dynamic_cast<const VertexSE3*>(_vertices[0]);

      const VertexSE3* v2 = dynamic_cast<const VertexSE3*>(_vertices[1]);

      // const VertexSBAPointXYZ* v2 = dynamic_cast<const VertexSBAPointXYZ*>(_vertices[1]);

      Vector3D delta = v1->estimate().translation() - v2->estimate().translation();


      _error[0] = _measurement - delta.norm();

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
 




 class G2O_TYPES_API disz : public  BaseBinaryEdge<2, Vector2D, VertexSE3, VertexSE3> 
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    disz();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    // return the error estimate as a 2-vector
    void computeError()
    {
      // from <Point> to <Cam>
      const VertexSE3 *v1 = static_cast<const VertexSE3*>(_vertices[0]);
      const VertexSE3 *v2 = static_cast<const VertexSE3*>(_vertices[1]);

      Vector3D delta = v1->estimate().translation() - v2->estimate().translation();
      double delta1 = v1->estimate().translation()[2]-_measurement[1];


       _error[0] = _measurement[0] - delta.norm();
       _error[1] = pow(delta1,2);


    }

     virtual void setMeasurement(const Vector2D& m)

      { 
        _measurement = m;
      }


     virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& , OptimizableGraph::Vertex* ) 
    { 
      return 1.;
    }

     virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);
 };
 



};








//  class G2O_TYPES_API ydedge : public  BaseBinaryEdge<2, Vector2D, VertexSE3, VertexSE3> 
// {
//   public:
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//     ydedge();
//     virtual bool read(std::istream& is);
//     virtual bool write(std::ostream& os) const;

//     // return the error estimate as a 2-vector
//     void computeError()
//     {
//       // from <Point> to <Cam>
//       const VertexSE3 *v1 = static_cast<const VertexSE3*>(_vertices[0]);
//       const VertexSE3 *v2 = static_cast<const VertexSE3*>(_vertices[1]);

//       // calculate the projection

//       Vector3D delta = v1->estimate().translation() - v2->estimate().translation();

//       double delta1 = v1->estimate().translation()[1] - _measurement[1];

   
//       _error[0] = _measurement[0] - delta.norm();

//       _error[1] = fabs(_measurement - delta1);


//     }

//       virtual void setMeasurement(const Vector2D& m)

//       { 
//         _measurement = m;
//       }


//       virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& , OptimizableGraph::Vertex* ) 
//     { 
//       return 1.;
//     }

//      virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);
//     };


//     }




#endif