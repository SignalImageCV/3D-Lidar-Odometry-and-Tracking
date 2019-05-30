#pragma once
#include "../interfaces/FeatureExtractorInterface.hpp"

namespace Loam{

   class FeatureExtractor: FeatureExtractorInterface{

    public:
      FeatureExtractor() = default;

      FeatureExtractor(int nonsi );

      ~FeatureExtractor() = default;

      void computeSingleSmoothnessPaper(const std::vector<ScanPoint> & other_points, ScanPoint & point);
      void computeSmoothnessPaper( std::vector<ScanPoint> & points);
      void computeSingleSmoothnessMine(const std::vector<ScanPoint> & other_points, ScanPoint & point);
      void computeSmoothnessMine( std::vector<ScanPoint> & points);

      ScanPoint findMaxSmoothnessPoint( const  std::vector<ScanPoint> & points);
      ScanPoint findMinSmoothnessPoint( const  std::vector<ScanPoint> & points);

      std::list<ScanPoint> sortForDecreasingSmoothness(const  std::vector<ScanPoint> & points);
      std::list<ScanPoint> sortForIncreasingSmoothness(const  std::vector<ScanPoint> & points);
      std::vector<std::vector<ScanPoint>>  divideInSectors( const int num_sectors, const  std::vector<ScanPoint> & points);


  };
}

