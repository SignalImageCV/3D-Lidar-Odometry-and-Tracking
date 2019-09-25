#pragma once
#include <srrg_config/param_macros.h>
#include <srrg_property/property.h>

#include "./CorrespondenceFinderMatchables.hpp"

namespace Loam{

  class CorrespondenceFinderMatchablesBruteForce : public CorrespondenceFinderMatchables {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    // ia just in case
    using BaseType = CorrespondenceFinderMatchables;
    using ThisType = CorrespondenceFinderMatchablesBruteForce;
    using Scalar   = BaseType::TransformType::Scalar;

  public:

    //! @brief ctor
    CorrespondenceFinderMatchablesBruteForce() : BaseType() {
    }

    //! @brief dtor
    virtual ~CorrespondenceFinderMatchablesBruteForce() {
    }

    //! @brief resets the kdtrees and populates them
    void reset() final;

    //! @brief override of BaseType::compute (non overridable).
    //! in this case we have to populate the kdtrees from the fixed and then,
    //! for each matchable in the moving, query the trees according to the type of primitive.
    void compute() final;

  protected:

    bool _is_initialized = false;
 };

  using CorrespondenceFinderMatchablesBruteForcePtr= std::shared_ptr<CorrespondenceFinderMatchablesBruteForce>;

}
