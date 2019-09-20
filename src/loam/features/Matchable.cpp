#include "Matchable.hpp"

namespace Loam{

  Matchable::Matchable( 
      const Eigen::Vector3f & p_m,
      const Eigen::Matrix3f & R_m,
      const Eigen::Matrix3f & Omega_m):
    MatchableInterface( p_m, R_m, Omega_m)
  {
    m_className = "Matchable";
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
