#pragma once

#include <Eigen/Core>
#include <srrg_system_utils/system_utils.h>
#include <srrg_messages/instances.h>
#include "../MyMath.hpp"
#include "../Drawer.hpp"


using namespace srrg2_core;


namespace Loam{

  typedef struct eigenvaluesBasedStats_tag{
    float linearity;
    float planarity;
    float scattering;
    float omnivariance;
    float anisotropy;
    float eigenentropy;
    float sumOfEigenvalues;
    float changeOfCurvature;
    eigenvaluesBasedStats_tag():
      linearity(0.),
      planarity(0.),
      scattering(0.),
      omnivariance(0.),
      anisotropy(0.),
      eigenentropy(0.),
      sumOfEigenvalues(0.),
      changeOfCurvature(0.)
    {}
  }eigenvaluesBasedStats;



  class MatchableInterface{
    protected:
      Eigen::Vector3f p_m;
      Eigen::Matrix3f R_m;
      Eigen::Matrix3f Omega_m;

    public:

      eigenvaluesBasedStats stats;
      MatchableInterface(
          const Eigen::Vector3f & p_m,
          const Eigen::Matrix3f & R_m,
          const Eigen::Matrix3f & Omega_m):
        p_m( p_m), R_m( R_m), Omega_m( Omega_m)
    {}

      virtual ~MatchableInterface() = default;


      virtual float computeDistance(
          const  PointNormalColor3f & t_point) =0;

      virtual float computeEigenvalueConstraint() =0;

      virtual PointNormalColor3fVectorCloud drawMatchable(
          const float length,
          const float precision,
          const Vector3f & color
          ) =0;


      inline const Eigen::Vector3f & get_p_m() const { return p_m;};
      inline Eigen::Vector3f & get_p_m() { return p_m;};

      inline const Eigen::Matrix3f & get_R_m() const { return R_m;};
      inline Eigen::Matrix3f & get_R_m() { return R_m;};

      inline const Eigen::Matrix3f & get_Omega_m() const { return Omega_m;};
      inline Eigen::Matrix3f & get_Omega_m() { return Omega_m;};

  };

}


