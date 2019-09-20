#include "Point.hpp"

namespace Loam{

  Point::Point(const Eigen::Vector3f & p_m,
          const Eigen::Matrix3f & R_m,
          const Eigen::Matrix3f & Omega_m):
        Matchable(p_m, R_m, Omega_m)
     {
       m_className = "Point";
     }

  float Point::computeDistance(const  PointNormalColor3f & t_point){
    //todo
    float distance = -1;
    return distance;
  }

  float Point::computeEigenvalueConstraint(){
    //todo
    float constraint= 0;
    return constraint;
  }


 PointNormalColor3fVectorCloud Point::drawMatchable( const float length, const float precision ){
    PointNormalColor3fVectorCloud  drawingPoint;
    PointNormalColor3f p;
    p.coordinates() = p_m; 
    drawingPoint.push_back( p);
    return drawingPoint;
 }
      
 

}
