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
      points[i].setSmoothnees( c);
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
      points[i].setSmoothnees( c);
    }
  };



}




