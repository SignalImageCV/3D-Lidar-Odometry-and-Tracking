#include "Plane.hpp"

namespace Loam{

  Plane::Plane(const Eigen::Vector3f & p_m,
          const Eigen::Matrix3f & R_m,
          const Eigen::Matrix3f & Omega_m):
        Matchable(p_m, R_m, Omega_m)
     {
       m_className = "Plane";
     }

  float Plane::computeDistance(const  PointNormalColor3f & t_point){

    const  Eigen::Vector3f u = R_m.col(0).normalized();
    const  Eigen::Vector3f v = R_m.col(1).normalized();
    const  Eigen::Vector3f plane_normal =  u.cross(v);
    const float distance_lineCenter_point = ( t_point.coordinates() - p_m).norm();
    const Eigen::Vector3f unit_vec_lineCenter_point =  ( t_point.coordinates() - p_m).normalized();

    const  Eigen::Vector3f hortogonal_component =
      (unit_vec_lineCenter_point.dot( plane_normal)/ plane_normal.squaredNorm() )* plane_normal;

    const  Eigen::Vector3f projection_unit_vec_lineCenter_point_on_plane =
      unit_vec_lineCenter_point - hortogonal_component;

    const  Eigen::Vector3f unit_vec_projection =
      projection_unit_vec_lineCenter_point_on_plane.normalized();

    float dot_prod_between_unit_vecs= unit_vec_projection.dot(unit_vec_lineCenter_point);

    if ( dot_prod_between_unit_vecs < -1.f){
      dot_prod_between_unit_vecs = -1.f;
    }
    else if( dot_prod_between_unit_vecs > 1.f){
      dot_prod_between_unit_vecs = 1.f;
    }

    const float angle_between_unit_vecs = acos(  dot_prod_between_unit_vecs);

    float result =  distance_lineCenter_point * sin( angle_between_unit_vecs);
    
    return result;

    //return distance_lineCenter_point * sin( angle_between_unit_vecs);

 }

  float Plane::computeEigenvalueConstraint(){
    Eigen::Vector3f diag = Omega_m.diagonal();

    const float num = diag.z();
    const float denom = diag.x() + diag.y() + diag.z();
    return num/denom;
 }

  PointNormalColor3fVectorCloud Plane::drawMatchable(
      const float length,
      const float precision,
      const Vector3f & color
      ){
    Eigen::Vector3f diag = Omega_m.diagonal();
    
    PointNormalColor3fVectorCloud drawingPoints =
      Drawer::createPlane( p_m,
          R_m.col(0),
          R_m.col(1),
          length*diag.x(),
          length*diag.y(),
          precision*diag.x(),
          precision*diag.y(),
          color);

    return drawingPoints;
  }
 
}
