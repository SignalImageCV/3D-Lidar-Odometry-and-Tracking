#include "instances.h"
namespace Loam{

  using namespace srrg2_core;
  using namespace srrg2_slam_interfaces;

  void loam_aligner_registerTypes() {
    BOSS_REGISTER_CLASS(CustomAlignerSlice3d);
  }
}
