#pragma once
#include "Matchable.hpp"

namespace Loam{

   class Plane: public Matchable{

    public:
      Plane() = default;
      Plane(const Eigen::Vector3f & p_m,
          const Eigen::Matrix3f & R_m,
          const Eigen::Matrix3f & Omega_m);

      ~Plane() = default;

      float computeDistance(const  PointNormalColor3f & t_point);

      float computeEigenvalueConstraint();
 
  };
}

