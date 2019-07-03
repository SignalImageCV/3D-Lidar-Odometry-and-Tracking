#include "Line.hpp"

namespace Loam{

  Line::Line(const Eigen::Vector3f & p_m,
          const Eigen::Matrix3f & R_m,
          const Eigen::Matrix3f & Omega_m):
        Matchable(p_m, R_m, Omega_m)
     {}

  float Line::computeDistance(const  PointNormalColor3f & t_point){
    //if it is correct then, for the sake of optimization,pass the svd object in costruction

    const Eigen::Matrix3f sigma = R_m * Omega_m * R_m.transpose();
    const Eigen::JacobiSVD<Eigen::Matrix3f> svd(sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Vector3f ls_error = svd.solve( t_point.coordinates());
    return ls_error.norm();
  }

  float Line::computeEigenvalueConstraint(){
    Eigen::Vector3f diag = Omega_m.diagonal();
    const float num = diag.x() + diag.y();
    const float denom = diag.x() + diag.y() + diag.z();
    return num/denom;
  }
 

}
