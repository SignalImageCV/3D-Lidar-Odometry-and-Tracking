#include <gtest/gtest.h>
#include "../loam/DatasetManager.hpp"

using namespace testing;
namespace Loam{
  TEST( datasetProcessing , singleRow){
    string filename = "../../datasets/kitti_2011_09_30_drive_0020_synced.bag";
    DatasetManager dM( filename);

    PointNormalColor3fVectorCloud points =  dM.readMessageFromDataset();
    ASSERT_EQ( 121184, points.size());
    ASSERT_EQ( points[0].normal().x(), 0.);
    ASSERT_EQ( points[0].normal().y(), 0.);
    ASSERT_EQ( points[0].normal().z(), 0.);
  }
}

