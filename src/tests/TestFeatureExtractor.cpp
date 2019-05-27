#include "../loam/features/FeatureExtractor.hpp"
#include <gtest/gtest.h>
#include <stdio.h>

using namespace testing;
namespace Loam{

  TEST( FeatureExtractor, allDistancesEqual1){

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

    EXPECT_EQ( -1, points[0].getSmoothnees());
    EXPECT_EQ( -1, points[3].getSmoothnees());

    fE.computeSmoothness(points);
    //with the function of the smoothness of the paper need to change all 0 with ones

    EXPECT_EQ( 0, points[0].getSmoothnees());
    EXPECT_EQ( 0, points[1].getSmoothnees());
    EXPECT_EQ( 0, points[2].getSmoothnees());
    EXPECT_EQ( 0, points[3].getSmoothnees());
    EXPECT_EQ( 0, points[4].getSmoothnees());
    EXPECT_EQ( 0, points[5].getSmoothnees());

  }

  TEST( FeatureExtractor, edgePoint){

    FeatureExtractor fE = FeatureExtractor(0);

    std::vector<ScanPoint> points;
    points.reserve( 6);
    ScanPoint p1( 0,0, 3., 3., 0.);
    ScanPoint p2( 0,1, 2., 2., 1.);
    ScanPoint p3( 0,2, 1.2, 1.2, 1.2);
    ScanPoint p4( 0,3, 1., 1., 2.);
    ScanPoint p5( 0,4, 0., 0., 3.);
    points.push_back( p1);
    points.push_back( p2);
    points.push_back( p3);
    points.push_back( p4);
    points.push_back( p5);

    fE.computeSmoothness(points);
    std::cout << points[0].getSmoothnees()<< " 1 \n";
    std::cout << points[1].getSmoothnees()<< " 2 \n";
    std::cout << points[2].getSmoothnees()<< " 3 \n";
    std::cout << points[3].getSmoothnees()<< " 4 \n";
    std::cout << points[4].getSmoothnees()<< " 5 \n";

    EXPECT_TRUE(  points[2].getSmoothnees() > points[0].getSmoothnees());
    EXPECT_TRUE(  points[2].getSmoothnees() > points[1].getSmoothnees());
    EXPECT_TRUE(  points[2].getSmoothnees() > points[3].getSmoothnees());
    EXPECT_TRUE(  points[2].getSmoothnees() > points[4].getSmoothnees());

  }

}


