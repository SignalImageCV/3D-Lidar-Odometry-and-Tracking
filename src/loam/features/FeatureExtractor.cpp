#include "FeatureExtractor.hpp"

using namespace Eigen;
namespace Loam{

  FeatureExtractor::FeatureExtractor( int nonsi ):
    FeatureExtractorInterface( nonsi ){};

  void FeatureExtractor::computeSmoothness( std::vector<ScanPoint> & points){
    for(unsigned int i = 0; i<points.size();++i){
      Vector3f sum_distances = Vector3f::Zero();
      for(unsigned int j = 0; j<points.size();++i){
        if ( i != j){
          sum_distances += points[i].getCoords() - points[j].getCoords();
        }
      }
      float denom = points.size() * points[i].getCoords().norm();
      float c = sum_distances.norm() / denom ;
      points[i].setSmoothnees( c);
    }
  };

}




