#pragma once
#include "Matchable.hpp"

//#define SHARED(name) \
 //using name##Ptr = std::shared_ptr<name>; \
 //using name...Vector = std::vector<name...>; \
 
namespace Loam{

   class Point: public Matchable{

    public:

      Point(const Eigen::Vector3f & p_m,
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

  // SHARED(Point)
  using PointPtr= std::shared_ptr<Point>;
}

