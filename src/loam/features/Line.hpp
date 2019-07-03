#pragma once
#include "Matchable.hpp"

namespace Loam{

   class Line: public Matchable{

    public:
      Line() = default;

 
      Line(const Eigen::Vector3f & p_m,
          const Eigen::Matrix3f & R_m,
          const Eigen::Matrix3f & Omega_m);


      ~Line() = default;

      float computeDistance(const  PointNormalColor3f & t_point);

      float computeEigenvalueConstraint();
 

  };
}

