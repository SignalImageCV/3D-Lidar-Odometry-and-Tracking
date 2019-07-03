#include "Plane.hpp"

namespace Loam{

  Plane::Plane(const Eigen::Vector3f & p_m,
          const Eigen::Matrix3f & R_m,
          const Eigen::Matrix3f & Omega_m):
        Matchable(p_m, R_m, Omega_m)
     {}

  float Plane::computeDistance(const  PointNormalColor3f & t_point){

    const Eigen::Matrix3f sigma = R_m * Omega_m * R_m.transpose();
    const Eigen::JacobiSVD<Eigen::Matrix3f> svd(sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Vector3f ls_error = svd.solve( t_point.coordinates());
    return ls_error.norm();
  }

  float Plane::computeEigenvalueConstraint(){
    Eigen::Vector3f diag = Omega_m.diagonal();
    const float num = diag.x();
    const float denom = diag.x() + diag.y() + diag.z();
    return num/denom;
 }
 
}
