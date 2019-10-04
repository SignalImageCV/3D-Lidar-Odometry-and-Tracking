#include "Matchable.hpp"

namespace Loam{

  Matchable::Matchable( 
      const Eigen::Vector3f & p_m,
      const Eigen::Matrix3f & R_m,
      const Eigen::Matrix3f & Omega_m):
    MatchableInterface( p_m, R_m, Omega_m)
  {
    m_className = "Matchable";

    const Eigen::Vector3f eigenvalues= Omega_m.diagonal();
    const float e1 = eigenvalues.x();
    const float e2 = eigenvalues.y();
    const float e3 = eigenvalues.z();

    stats.linearity = (e1 -e2)/e1;
    stats.planarity = (e2 -e3)/e1;
    stats.scattering = e3/e1;
    stats.omnivariance = pow( e1*e2*e3 , 1./3.);
    stats.anisotropy = (e1-e3)/e1;
    stats.eigenentropy = - e1*log(e1) - e2*log(e2) - e3*log(e3);
    stats.sumOfEigenvalues = e1 + e2 + e3;
    stats.changeOfCurvature = e3/ ( e1+e2+e3);

  }


  float Matchable::computeResidualError(
      const PointNormalColor3fVectorCloud & t_points){
    float cumulative_error = 0;
    const int c = t_points.size();
    for ( auto & p: t_points){
      cumulative_error += computeDistance( p);
    }
    //std::cout<< "residual error : cumulative  = " << cumulative_error<< " number of points = "<< c<< "\n" ;
    return cumulative_error/ c;
  }

//  float Matchable::computeDistance(const  PointNormalColor3f & t_point){
//    std::cout<< "Calling a method that should be overriden by its subclasses\n";
//    return 0;
//  }
//
//  float Matchable::computeEigenvalueConstraint(){
//    std::cout<< "Calling a method that should be overriden by its subclasses\n";
//    return 0;
//  }
//
//  PointNormalColor3fVectorCloud Matchable::drawMatchable( const float length, const float precision ){
//    std::cout<< "Calling a method that should be overriden by its subclasses\n";
//    std::cout<< "This is it\n";
//    PointNormalColor3fVectorCloud empty;
//    return empty;
//  }
}
