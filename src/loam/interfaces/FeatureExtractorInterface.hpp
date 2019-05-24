#pragma once
#include <vector>
#include "../ScanPoint.hpp"

namespace Loam{

  class  FeatureExtractorInterface{
    public:
      FeatureExtractorInterface() = default;

      FeatureExtractorInterface(int nonsi){};

      ~FeatureExtractorInterface() = default;

      //Function that given a cloud of points in a vertical slice
      // (called scan plane) e.g. produced in a single instant from
      // a lidar, computes the smoothness c of every one
      // it updates the c value inside the struct of each point
      virtual  void computeSmoothness( std::vector<ScanPoint> & points) =0;

  };
}

