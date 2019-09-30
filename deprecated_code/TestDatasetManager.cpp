#include <gtest/gtest.h>
#include "../loam/DatasetManager.hpp"

using namespace testing;
using namespace srrg2_core;
namespace Loam{
  TEST( datasetProcessing , singleRow){
    string filename = "../../datasets/kitti_2011_09_30_drive_0020_synced.bag";
    messages_registerTypes();
    srrgInit( argc, argv, "hi");

    DatasetManager dM( filename);


    PointNormalColor3fVectorCloud points =  dM.readMessageFromDataset();
    ASSERT_EQ( 121184, points.size());
    ASSERT_EQ( points[0].normal().x(), 0.);
    ASSERT_EQ( points[0].normal().y(), 0.);
    ASSERT_EQ( points[0].normal().z(), 0.);
  }
}

