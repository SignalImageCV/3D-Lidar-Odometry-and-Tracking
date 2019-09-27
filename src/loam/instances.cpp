#include "instances.h"
namespace Loam{

  using namespace srrg2_core;
  using namespace srrg2_slam_interfaces;

  void loam_registerTypes() {
    point_cloud_registerTypes();


    BOSS_REGISTER_CLASS(CorrespondenceFinderMatchables);
    BOSS_REGISTER_CLASS(CorrespondenceFinderMatchablesBruteForce);
    BOSS_REGISTER_CLASS(CorrespondenceFinderMatchablesKDTree);
  //  BOSS_REGISTER_CLASS(CustomMatchable_);
    BOSS_REGISTER_CLASS(CustomMeasurementAdaptor);
    BOSS_REGISTER_BLOB(CustomMatchabledVectorData);
    BOSS_REGISTER_BLOB(CustomMatchablefVectorData);
  }

}
