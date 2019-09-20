#include "Line.hpp"

namespace Loam{

  Line::Line(const Eigen::Vector3f & p_m,
          const Eigen::Matrix3f & R_m,
          const Eigen::Matrix3f & Omega_m):
        Matchable(p_m, R_m, Omega_m)
     {
       m_className = "Line";
     }


  float Line::computeDistance(const  PointNormalColor3f & t_point){
    const float distance_lineCenter_point = ( t_point.coordinates() - p_m).norm();
    const Eigen::Vector3f unit_vec_lineCenter_point =  ( t_point.coordinates() - p_m).normalized();
    const Eigen::Vector3f unit_vec_line_direction = R_m.col(0).normalized();
    const float angle_between_unit_vecs =
      acos( unit_vec_line_direction.dot( unit_vec_lineCenter_point));
    return distance_lineCenter_point * sin( angle_between_unit_vecs);
  }

  float Line::computeEigenvalueConstraint(){
    Eigen::Vector3f diag = Omega_m.diagonal();
    const float num = diag.y() + diag.z();
    const float denom = diag.x() + diag.y() + diag.z();
    return num/denom;
  }
 
  PointNormalColor3fVectorCloud Line::drawMatchable( const float length, const float precision ){
    PointNormalColor3fVectorCloud drawingPoints =
      Drawer::createLine( p_m, R_m.col(0),length, precision );
    return drawingPoints;
  }

}
