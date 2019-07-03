#pragma once
#include "../interfaces/MatchableInterface.hpp"

namespace Loam{

  class Matchable : public MatchableInterface{

    public:

      Matchable() = default;

      Matchable( 
          const Eigen::Vector3f & p_m,
          const Eigen::Matrix3f & R_m,
          const Eigen::Matrix3f & Omega_m);

      float computeResidualError(const PointNormalColor3fVectorCloud & t_points);

      float computeDistance(const  PointNormalColor3f & t_point);

      float computeEigenvalueConstraint();

  
      ~Matchable() = default;
  };
}


