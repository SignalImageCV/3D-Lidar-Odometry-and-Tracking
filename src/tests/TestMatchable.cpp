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

    Line line = Line( mu, R, Omega);

    const Eigen::Vector3f p_query_1( 1.,-2.,0.);
    const Eigen::Vector3f p_query_2( 6.,2.,0.);
    const Eigen::Vector3f p_query_3( 4.,0.,-4.);
    const Eigen::Vector3f p_query_4( 7.,1.,3.);
    PointNormalColor3f point_1;
    PointNormalColor3f point_2;
    PointNormalColor3f point_3;
    PointNormalColor3f point_4;
    point_1.coordinates() = p_query_1;
    point_2.coordinates() = p_query_2;
    point_3.coordinates() = p_query_3;
    point_4.coordinates() = p_query_4;
    ASSERT_NEAR( line.computeDistance(point_1), 3 , 1e-3);
    ASSERT_NEAR( line.computeDistance(point_2), 2 , 1e-3);
    ASSERT_NEAR( line.computeDistance(point_3), 4 , 1e-3);
    ASSERT_NEAR( line.computeDistance(point_4), sqrt(18) , 1e-3);
  }

  TEST( matchable, planeDistance){

    const Eigen::Vector3f p( 4.,0.,0.);
    const Eigen::Vector3f u( 0.,0.,1.);
    const Eigen::Vector3f v( 0.,1.,0.);
    
    PointNormalColor3fVectorCloud plane_cloud =  Drawer::createPlane( p, u, v, 10, 8, 1, 1);
    vector<Eigen::Vector3f> points;
    points.reserve( plane_cloud.size());
    for( auto & p: plane_cloud){
      points.push_back(p.coordinates());
    }

    const Eigen::Vector3f mu =  MyMath::computeMuGaussian( points);
    const Eigen::Matrix3f sigma =  MyMath::computeSigmaGaussian( points, mu);

    Eigen::JacobiSVD<Eigen::Matrix3f> svd(sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);

    const Eigen::Matrix3f R = svd.matrixU();
    const Eigen::Vector3f s = svd.singularValues();
    Eigen::Matrix3f Omega;
    Omega << s.x() ,0, 0,
          0, s.y(), 0,
          0, 0, s.z();

    Plane plane = Plane( mu, R, Omega);

    const Eigen::Vector3f p_query_1( 1.,-2.,0.);
    const Eigen::Vector3f p_query_2( 6.,2.,0.);
    const Eigen::Vector3f p_query_3( 4.,0.,-4.);
    const Eigen::Vector3f p_query_4( 7.,1.,3.);
    PointNormalColor3f point_1;
    PointNormalColor3f point_2;
    PointNormalColor3f point_3;
    PointNormalColor3f point_4;
    point_1.coordinates() = p_query_1;
    point_2.coordinates() = p_query_2;
    point_3.coordinates() = p_query_3;
    point_4.coordinates() = p_query_4;
    ASSERT_NEAR( plane.computeDistance(point_1), 3 , 1e-3);
    ASSERT_NEAR( plane.computeDistance(point_2), 2 , 1e-3);
    ASSERT_NEAR( plane.computeDistance(point_3), 0 , 1e-3);
    ASSERT_NEAR( plane.computeDistance(point_4), 3 , 1e-3);
  }

}


