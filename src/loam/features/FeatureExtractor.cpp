#include "FeatureExtractor.hpp"

using namespace Eigen;
namespace Loam{

  FeatureExtractor::FeatureExtractor( int nonsi ):
    FeatureExtractorInterface( nonsi ){};

  void FeatureExtractor::computeSmoothnessPaper( std::vector<ScanPoint> & points){
    for(unsigned int i = 0; i<points.size();++i){
      Vector3f sum_distances = Vector3f::Zero();
      for(unsigned int j = 0; j<points.size();++j){
        if ( i != j){
          sum_distances += points[i].getCoords() - points[j].getCoords();
        }
      }
      float denom = points.size() * points[i].getCoords().norm();
      float c = sum_distances.norm() / denom ;
      points[i].setSmoothness( c);
    }
  };

  void FeatureExtractor::computeSmoothness( std::vector<ScanPoint> & points){
    for(unsigned int i = 0; i<points.size();++i){
      float sum_distances = 0;
      for(unsigned int j = 0; j<points.size();++j){
        if ( i != j){
          sum_distances += points[i].getCoords().norm() - points[j].getCoords().norm();
        }
      }
      float denom = points.size() * points[i].getCoords().norm();
      float c = abs(sum_distances) / denom ;
      points[i].setSmoothness( c);
    }
  };

  ScanPoint FeatureExtractor::findMaxSmoothnessPoint( const  std::vector<ScanPoint> & points){
    float curr_max_value = points[0].getSmoothness();
    ScanPoint curr_max_point = points[0];

    for( auto& p: points){
      if( p.getSmoothness() > curr_max_value){
        curr_max_point = p;
      }
    }
    return curr_max_point;
  }

  ScanPoint FeatureExtractor::findMinSmoothnessPoint( const  std::vector<ScanPoint> & points){
    float curr_min_value = points[0].getSmoothness();
    ScanPoint curr_min_point = points[0];

    for( auto& p: points){
      if( p.getSmoothness() < curr_min_value){
        curr_min_point = p;
      }
    }
    return curr_min_point;
  }

  std::list<ScanPoint> FeatureExtractor::sortForDecreasingSmoothness(const  std::vector<ScanPoint> & points){
    std::list<ScanPoint> point_list( points.begin(), points.end()); 
    point_list.sort([](const ScanPoint & sp1, const ScanPoint & sp2)
        {
          return sp1.getSmoothness() > sp2.getSmoothness();
        });
    return point_list;
  }
 
  std::list<ScanPoint> FeatureExtractor::sortForIncreasingSmoothness(const  std::vector<ScanPoint> & points){
    std::list<ScanPoint> point_list( points.begin(), points.end()); 
    point_list.sort([](const ScanPoint & sp1, const ScanPoint & sp2)
        {
          return sp1.getSmoothness() < sp2.getSmoothness();
        });
    return point_list;
  }
 

}




