#include <gtest/gtest.h>
#include <stdio.h>
#include "../loam/features/Line.hpp"
#include "../loam/features/Plane.hpp"
#include "../loam/MyMath.hpp"

namespace Loam{
  using namespace testing;

  TEST( matchable, lineDistance){
    const Eigen::Vector3f p1( 4.,1.,0.);
    const Eigen::Vector3f p2( 4.,3.,0.);
    const Eigen::Vector3f p3( 4.,5.,0.);
    const Eigen::Vector3f p4( 4.,-1.,0.);
    const Eigen::Vector3f p5( 4.,-3.,0.);
    const Eigen::Vector3f p6( 4.,-5.,0.);


    const std::vector<Eigen::Vector3f> points={p1,p2,p3,p4,p5,p6};

    const Eigen::Vector3f mu =  MyMath::computeMuGaussian( points);
    const Eigen::Matrix3f sigma =  MyMath::computeSigmaGaussian( points, mu);

    Eigen::JacobiSVD<Eigen::Matrix3f> svd(sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);

    const Eigen::Matrix3f R = svd.matrixU();
    const Eigen::Vector3f s = svd.singularValues();
    Eigen::Matrix3f Omega;
    Omega << s.x() ,0, 0,
          0, s.y(), 0,
          0, 0, s.z();

    Line l = Line( mu, R, Omega);

    const Eigen::Vector3f p_query( 1.,-2.,0.);
    PointNormalColor3f point;
    point.coordinates() = p_query;
    ASSERT_NEAR( l.computeDistance(point), 3 , 1e-3);
  }
}


