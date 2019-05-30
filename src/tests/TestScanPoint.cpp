#include "../loam/ScanPoint.hpp"
#include <gtest/gtest.h>
#include <stdio.h>

using namespace testing;
namespace Loam{
  TEST( ScanPoint, equals){
    ScanPoint p1( 0,0, 1., -1., -1.);
    ScanPoint p2( 0,0, 1., -1., -1.);
    ScanPoint p3( 0,1, 1., -1., -1.);
    ScanPoint p4( 1,1, 1., -1., -1.);
    ScanPoint p5( 0,0, -1., -1., -1.);
    ScanPoint p6( 0,0, 1., 1., -1.);
    ScanPoint p7( 0,0, 1., -1., 1.);
    
    ASSERT_TRUE( p1 == p2);
    ASSERT_TRUE( p1 != p3);
    ASSERT_TRUE( p1 != p4);
    ASSERT_TRUE( p1 != p5);
    ASSERT_TRUE( p1 != p6);
    ASSERT_TRUE( p1 != p7);
  }

 
  TEST( ScanPoint, generateCubeSampledScanPointsLittle){

    std::vector<ScanPoint> truth;
    truth.reserve( 8);

    ScanPoint p1( -1,0, -1., -1., -1.);
    ScanPoint p2( -1,1, 1., -1., -1.);
    ScanPoint p3( -1,2, -1., 1., -1.);
    ScanPoint p4( -1,3, 1., 1., -1.);
    ScanPoint p5( -1,4, -1., -1., 1.);
    ScanPoint p6( -1,5, 1., -1., 1.);
    ScanPoint p7( -1,6, -1., 1., 1.);
    ScanPoint p8( -1,7, 1., 1., 1.);
    truth.push_back( p1);
    truth.push_back( p2);
    truth.push_back( p3);
    truth.push_back( p4);
    truth.push_back( p5);
    truth.push_back( p6);
    truth.push_back( p7);
    truth.push_back( p8);

    Eigen::Vector3f center( 0, 0, 0);
    float length_edge = 2;
    int precision = 1;
    std::vector<ScanPoint> result = ScanPoint::generateCubeSampledScanPoints(
        center, length_edge,precision);


    ASSERT_EQ( result.size(), truth.size());

    std::vector<ScanPoint>::iterator it1=result.begin();
    std::vector<ScanPoint>::iterator it2=truth.begin();

    while( it1 != result.end() &&
      it2 != truth.end()){
      ASSERT_TRUE( *it1 == *it2);
      ++it1, ++it2;
    }
  }
}


