#include "../loam/features/FeatureExtractor.hpp"
#include <gtest/gtest.h>

using namespace testing;
namespace Loam{

  TEST( FeatureExtractor, allDistancesEqual){

    FeatureExtractor fE = FeatureExtractor(0);

    std::vector<ScanPoint> points;
    points.reserve( 6);
    ScanPoint p1( 0,0, 1., 0., 0.);
    ScanPoint p2( 0,1, -1., 0., 0.);
    ScanPoint p3( 0,2, 0., 1., 0.);
    ScanPoint p4( 0,3, 0., -1., 0.);
    ScanPoint p5( 0,4, 0., 0., 1.);
    ScanPoint p6( 0,5, 0., 0., -1.);
    points.push_back( p1);
    points.push_back( p2);
    points.push_back( p3);
    points.push_back( p4);
    points.push_back( p5);
    points.push_back( p6);

    EXPECT_EQ( -1, p1.getSmoothnees());
    EXPECT_EQ( -1, p4.getSmoothnees());

    fE.computeSmoothness(points);

    EXPECT_EQ( 0, p1.getSmoothnees());
    EXPECT_EQ( 0, p2.getSmoothnees());
    EXPECT_EQ( 0, p3.getSmoothnees());
    EXPECT_EQ( 0, p4.getSmoothnees());
    EXPECT_EQ( 0, p5.getSmoothnees());
    EXPECT_EQ( 0, p6.getSmoothnees());

  }
}


