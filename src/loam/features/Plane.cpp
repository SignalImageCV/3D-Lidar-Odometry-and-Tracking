#include "Plane.hpp"

namespace Loam{

  Plane::Plane(const Eigen::Vector3f & p_m,
          const Eigen::Matrix3f & R_m,
          const Eigen::Matrix3f & Omega_m):
        Matchable(p_m, R_m, Omega_m)
     {}

  float Plane::computeDistance(const  PointNormalColor3f & t_point){

    const  Eigen::Vector3f u = R_m.row(0).normalized();
    const  Eigen::Vector3f v = R_m.row(1).normalized();
    const  Eigen::Vector3f plane_normal =  u.cross(v);
    const float distance_lineCenter_point = ( t_point.coordinates() - p_m).norm();
    const Eigen::Vector3f unit_vec_lineCenter_point =  ( t_point.coordinates() - p_m).normalized();

    const  Eigen::Vector3f hortogonal_component =
      (unit_vec_lineCenter_point.dot( plane_normal)/ plane_normal.squaredNorm() )* plane_normal;

    const  Eigen::Vector3f projection_unit_vec_lineCenter_point_on_plane =
      unit_vec_lineCenter_point - hortogonal_component;

    const  Eigen::Vector3f unit_vec_projection =
      projection_unit_vec_lineCenter_point_on_plane.normalized();

    const float angle_between_unit_vecs =
      acos( unit_vec_projection.dot(unit_vec_lineCenter_point));

    return distance_lineCenter_point * sin( angle_between_unit_vecs);

 }

  float Plane::computeEigenvalueConstraint(){
    Eigen::Vector3f diag = Omega_m.diagonal();
    const float num = diag.x();
    const float denom = diag.x() + diag.y() + diag.z();
    return num/denom;
 }
 
}
