#pragma once

#include "Matchable.hpp"

namespace Loam{

   class Line: public Matchable{

    public:
 
      Line(const Eigen::Vector3f & p_m,
          const Eigen::Matrix3f & R_m,
          const Eigen::Matrix3f & Omega_m);


      float computeDistance(const  PointNormalColor3f & t_point) override;

      float computeEigenvalueConstraint() override;
 
     PointNormalColor3fVectorCloud drawMatchable(
          const float length,
          const float precision,
          const Vector3f & color
          ) override;

  };

  using LinePtr= std::shared_ptr<Line>;
}

