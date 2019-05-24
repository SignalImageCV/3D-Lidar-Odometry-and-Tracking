#pragma once
#include "../interfaces/FeatureExtractorInterface.hpp"

namespace Loam{

   class FeatureExtractor: FeatureExtractorInterface{

    public:
      FeatureExtractor() = default;

      FeatureExtractor(int nonsi );

      ~FeatureExtractor() = default;

      void computeSmoothness( std::vector<ScanPoint> & points);


  };
}

