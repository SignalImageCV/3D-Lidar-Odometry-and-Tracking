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

    ASSERT_EQ( -1, points[0].getSmoothness());
    ASSERT_EQ( -1, points[3].getSmoothness());

    fE.computeSmoothness(points);
    //with the function of the smoothness of the paper need to change all 0 with ones

    ASSERT_EQ( 0, points[0].getSmoothness());
    ASSERT_EQ( 0, points[1].getSmoothness());
    ASSERT_EQ( 0, points[2].getSmoothness());
    ASSERT_EQ( 0, points[3].getSmoothness());
    ASSERT_EQ( 0, points[4].getSmoothness());
    ASSERT_EQ( 0, points[5].getSmoothness());

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
    std::cout << points[0].getSmoothness()<< " 1 \n";
    std::cout << points[1].getSmoothness()<< " 2 \n";
    std::cout << points[2].getSmoothness()<< " 3 \n";
    std::cout << points[3].getSmoothness()<< " 4 \n";
    std::cout << points[4].getSmoothness()<< " 5 \n";

    ASSERT_TRUE(  points[2].getSmoothness() > points[0].getSmoothness());
    ASSERT_TRUE(  points[2].getSmoothness() > points[1].getSmoothness());
    ASSERT_TRUE(  points[2].getSmoothness() > points[3].getSmoothness());
    ASSERT_TRUE(  points[2].getSmoothness() > points[4].getSmoothness());

  }

  TEST( FeatureExtractor, maxAndMinSmoothnessPoint){

    FeatureExtractor fE = FeatureExtractor(0);

    std::vector<ScanPoint> points;
    points.reserve( 6);
    ScanPoint p1( 0,0, 3., 3., 0.);
    p1.setSmoothness(1);
    ScanPoint p2( 0,1, 2., 2., 1.);
    p2.setSmoothness(12);
    ScanPoint p3( 0,2, 1.2, 1.2, 1.2);
    p3.setSmoothness(8);
    ScanPoint p4( 0,3, 1., 1., 2.);
    p4.setSmoothness(2);
    ScanPoint p5( 0,4, 0., 0., 3.);
    p5.setSmoothness(0);
    points.push_back( p1);
    points.push_back( p2);
    points.push_back( p3);
    points.push_back( p4);
    points.push_back( p5);

    ScanPoint min_p = fE.findMinSmoothnessPoint( points);
    ScanPoint max_p = fE.findMaxSmoothnessPoint( points);
    ASSERT_EQ( 4, min_p.getIndexInSweep());
    ASSERT_EQ( 1, max_p.getIndexInSweep());

  }

  TEST( FeatureExtractor, sortForIncreasingSmoothness){

    FeatureExtractor fE = FeatureExtractor(0);

    std::vector<ScanPoint> points;
    points.reserve( 6);
    ScanPoint p1( 0,0, 3., 3., 0.);
    p1.setSmoothness(   8   );
    ScanPoint p2( 0,1, 2., 2., 1.);
    p2.setSmoothness(  12   );
    ScanPoint p3( 0,2, 1.2, 1.2, 1.2);
    p3.setSmoothness(   8   );
    ScanPoint p4( 0,3, 1., 1., 2.);
    p4.setSmoothness(   2   );
    ScanPoint p5( 0,4, 0., 0., 3.);
    p5.setSmoothness(   0   );
    points.push_back( p1);
    points.push_back( p2);
    points.push_back( p3);
    points.push_back( p4);
    points.push_back( p5);

    std::list<ScanPoint> increasing_ordered_truth;
    increasing_ordered_truth.push_back(p5);
    increasing_ordered_truth.push_back(p4);
    increasing_ordered_truth.push_back(p1);
    increasing_ordered_truth.push_back(p3);
    increasing_ordered_truth.push_back(p2);

    std::list<ScanPoint> increasing_ordered_list= fE.sortForIncreasingSmoothness( points);
    ASSERT_EQ( increasing_ordered_list.size(), increasing_ordered_truth.size());

 
    std::list<ScanPoint>::iterator it1=increasing_ordered_truth.begin();
    std::list<ScanPoint>::iterator it2=increasing_ordered_list.begin();

    while( it1 != increasing_ordered_truth.end() &&
      it2 != increasing_ordered_list.end()){
      ASSERT_EQ( it1->getIndexInSweep(),it2->getIndexInSweep());
      ++it1, ++it2;
    }

  }

  TEST( FeatureExtractor, divideInSectorsFew){

    FeatureExtractor fE = FeatureExtractor(0);

    std::vector<ScanPoint> points;
    points.reserve( 5);
    ScanPoint p1( 0,0, 1., 0., 0.);
    ScanPoint p2( 0,1, -1., 0., 0.);
    ScanPoint p3( 0,2, 0., 1., 0.);
    ScanPoint p4( 0,3, 0., -1., 0.);
    ScanPoint p5( 0,4, 0., 0., 1.);
    points.push_back( p1);
    points.push_back( p2);
    points.push_back( p3);
    points.push_back( p4);
    points.push_back( p5);

    std::vector<std::vector<ScanPoint>> result = fE.divideInSectors( 4, points);

    ASSERT_EQ( result.size(), 4);
    ASSERT_EQ( result[0].size(), 1);
    ASSERT_EQ( result[1].size(), 1);
    ASSERT_EQ( result[2].size(), 1);
    ASSERT_EQ( result[3].size(), 2);
    }

  TEST( FeatureExtractor, divideInSectorsMany){

    FeatureExtractor fE = FeatureExtractor(0);

    std::vector<ScanPoint> points;
    int num_points = 123;
    points.reserve( num_points);
    for(unsigned int j = 0; j<num_points;++j){
      ScanPoint p( 0,0, 1., 0., 0.);
      points.push_back( p);
    }

    std::vector<std::vector<ScanPoint>> result = fE.divideInSectors( 4, points);

    ASSERT_EQ( result.size(), 4);
    ASSERT_EQ( result[0].size(), 30);
    ASSERT_EQ( result[1].size(), 30);
    ASSERT_EQ( result[2].size(), 30);
    ASSERT_EQ( result[3].size(), 33);
  }

}


