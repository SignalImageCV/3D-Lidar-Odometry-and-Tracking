#include "./CustomAlignerSlice3d.hpp"

namespace Loam{
  using namespace srrg2_solver;

  CustomAlignerSlice3d::CustomAlignerSlice3d() {
    param_fixed_slice_name.setValue("customMatchables");
    param_moving_slice_name.setValue("customMatchables");
    param_robustifier.setValue(std::shared_ptr<RobustifierSaturated>(new RobustifierSaturated()));
    param_robustifier->setName("aligner_robustifier");
    setName("custom_aligner_slice_3d");
  }

  void CustomAlignerSlice3d::setupFactor() {
  }

}
