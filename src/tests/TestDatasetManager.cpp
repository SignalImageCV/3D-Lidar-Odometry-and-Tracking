#include "../loam/DatasetManager.hpp"
#include <gtest/gtest.h>

using namespace testing;

namespace Loam{

  TEST( datasetProcessing , singleRow){
    string filename = "/home/dinies/temp/trial/tuttty.boss";
    DatasetManager dM( filename);
    vector<scanPoint> points = dM.readMessageFromDataset();

    EXPECT_EQ( 0, points[0].index_ofSweep);
    EXPECT_EQ( 121184, points.size());
  }
}


