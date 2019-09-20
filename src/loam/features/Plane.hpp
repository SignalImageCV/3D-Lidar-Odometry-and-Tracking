#pragma once
#include "Matchable.hpp"

namespace Loam{

   class Plane: public Matchable{

    public:
      Plane(const Eigen::Vector3f & p_m,
          const Eigen::Matrix3f & R_m,
          const Eigen::Matrix3f & Omega_m);

      float computeDistance(const  PointNormalColor3f & t_point) override;

      float computeEigenvalueConstraint() override;
 
      PointNormalColor3fVectorCloud drawMatchable( const float length, const float precision ) override;
  };

  using PlanePtr = std::shared_ptr<Plane>;
}

