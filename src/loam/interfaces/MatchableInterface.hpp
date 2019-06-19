#pragma once
#include <Eigen/Core>

namespace Loam{
  class MatchableInterface{
    protected:
      Eigen::Vector3f p_m;
      Eigen::Matrix3f R_m;
      Eigen::Matrix3f Omega_m;

    public:

      MatchableInterface() = default;

      ~MatchableInterface() = default;
  };

}


