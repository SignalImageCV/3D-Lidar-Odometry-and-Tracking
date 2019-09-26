#pragma once

// add the two files in cmakelists 
//since i removed them to debug undef ref
//
//
//

//#include <srrg_slam_interfaces/multi_aligner_slice.h>

#include <srrg_slam_interfaces/instances.h>
#include <srrg_solver/solver_core/instances.h>
#include <srrg_solver/SE3/instances.h>

//#include <srrg_solver/SE3/se3_matchable2matchable_error_factor.h>

//#include "../features/CustomMatchable.hpp"
#include "../matcher/CorrespondenceFinderMatchablesKDtree.hpp"

namespace Loam{

  class CustomAlignerSlice3d : public srrg2_slam_interfaces::MultiAlignerSlice_<
                           srrg2_solver::SE3Matchable2MatchableErrorFactorNoInfo,
                           CustomMatchablefVector,
                           CustomMatchablefVector> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   
    // how to force the usage of my correspondence finder ?? my guess
    using CorrespondenceFinderType = CorrespondenceFinderMatchablesKDTree;
    //using CorrespondenceFinderType = CorrespondenceFinder_<EstimateType, FixedType, MovingType>;

    CustomAlignerSlice3d();
    virtual ~CustomAlignerSlice3d(){};
    virtual void setupFactor() override;
  };

  using CustomAlignerSlice3dPtr = std::shared_ptr<CustomAlignerSlice3d>;

}

