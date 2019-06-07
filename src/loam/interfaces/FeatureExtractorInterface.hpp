#pragma once
#include <vector>
#include <list>
#include "../MyMath.hpp"
#include "../features/Matchable.hpp"

#include <srrg_boss/id_context.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_boss/serializer.h>
#include <srrg_boss/deserializer.h>
#include <srrg_messages/instances.h>


#include <iostream>

namespace Loam{
  using namespace std;
  using namespace srrg2_core;


  class  FeatureExtractorInterface{

    public:
      FeatureExtractorInterface() = default;

      ~FeatureExtractorInterface() = default;
      
     virtual vector<Matchable> extractFeatures() = 0;


  };
}

