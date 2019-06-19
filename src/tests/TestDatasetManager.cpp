#include <gtest/gtest.h>
#include "../loam/DatasetManager.hpp"

using namespace testing;
namespace Loam{
  TEST( datasetProcessing , singleRow){
    string filename = "/home/dinies/temp/trial/tuttty.boss";
    DatasetManager dM( filename);

    PointNormalColor3fVectorCloud points =  dM.readMessageFromDataset();
    ASSERT_EQ( 121184, points.size());
    ASSERT_EQ( points[0].normal().x(), 0.);
    ASSERT_EQ( points[0].normal().y(), 0.);
    ASSERT_EQ( points[0].normal().z(), 0.);
  }
}

