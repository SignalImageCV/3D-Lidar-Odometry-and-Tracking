#include <gtest/gtest.h>
#include "../loam/DatasetManager.hpp"

using namespace testing;
namespace Loam{
  TEST( datasetProcessing , singleRow){
    string filename = "/home/dinies/temp/trial/tuttty.boss";
    DatasetManager dM( filename);
    vector<ScanPoint> points = dM.readMessageFromDataset();
    ASSERT_EQ( 0, points[0].getIndexInSweep());
    ASSERT_EQ( 121184, points.size());
  }
}

