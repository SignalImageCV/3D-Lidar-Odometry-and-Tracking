#include "instances.h"
#include <srrg_slam_interfaces/multi_aligner_slice_impl.cpp>
namespace Loam{

  using namespace srrg2_core;
  using namespace srrg2_slam_interfaces;

  namespace dummy{
    static CustomAlignerSlice3d dummyAlign;
  }

  void loam_aligner_registerTypes() {
    BOSS_REGISTER_CLASS(CustomAlignerSlice3d);
  }
}
