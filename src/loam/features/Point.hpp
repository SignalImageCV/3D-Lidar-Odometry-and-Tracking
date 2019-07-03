#pragma once
#include "Matchable.hpp"

namespace Loam{

   class Point: public Matchable{

    public:
      Point() = default;

      Point(const Eigen::Vector3f & p_m,
          const Eigen::Matrix3f & R_m,
          const Eigen::Matrix3f & Omega_m);

      ~Point() = default;

      float computeDistance(const  PointNormalColor3f & t_point);

      float computeEigenvalueConstraint();
 
  };
}

