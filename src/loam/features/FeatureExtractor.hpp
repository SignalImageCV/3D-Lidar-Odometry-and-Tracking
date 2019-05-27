#pragma once
#include "../interfaces/FeatureExtractorInterface.hpp"

namespace Loam{

   class FeatureExtractor: FeatureExtractorInterface{

    public:
      FeatureExtractor() = default;

      FeatureExtractor(int nonsi );

      ~FeatureExtractor() = default;

      void computeSmoothnessPaper( std::vector<ScanPoint> & points);
      void computeSmoothness( std::vector<ScanPoint> & points);


  };
}

