#pragma once

#include <Eigen/Core>
#include <srrg_system_utils/system_utils.h>
#include <srrg_messages/instances.h>
#include "../MyMath.hpp"


using namespace srrg2_core;


namespace Loam{
  class MatchableInterface{
    protected:
      Eigen::Vector3f p_m;
      Eigen::Matrix3f R_m;
      Eigen::Matrix3f Omega_m;

    public:

      MatchableInterface() = default;
      MatchableInterface(
          const Eigen::Vector3f & p_m,
          const Eigen::Matrix3f & R_m,
          const Eigen::Matrix3f & Omega_m):
        p_m( p_m), R_m( R_m), Omega_m( Omega_m)
    {}

      ~MatchableInterface() = default;

      virtual float computeDistance(
          const  PointNormalColor3f & t_point) =0;

      virtual float computeEigenvalueConstraint() =0;
  };

}


