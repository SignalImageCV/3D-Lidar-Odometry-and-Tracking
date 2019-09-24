#pragma once
#include "../interfaces/MatchableInterface.hpp"

namespace Loam{

  class Matchable : public MatchableInterface{
    protected:
      std::string m_className;

    public:

      Matchable( 
          const Eigen::Vector3f & p_m,
          const Eigen::Matrix3f & R_m,
          const Eigen::Matrix3f & Omega_m);


      float computeResidualError(const PointNormalColor3fVectorCloud & t_points);

      virtual float computeDistance(
          const  PointNormalColor3f & t_point) =0;

      virtual float computeEigenvalueConstraint() =0;

      virtual PointNormalColor3fVectorCloud drawMatchable(
          const float length,
          const float precision,
          const Vector3f & color
          ) =0;

      inline const std::string  get_ClassName() const { return m_className;};

  };

  using MatchablePtr = std::shared_ptr<Matchable>;
  using MatchablePtrVec = std::vector< MatchablePtr>;
  using MatchablePtrVecPtr = std::shared_ptr<MatchablePtrVec>;
}


