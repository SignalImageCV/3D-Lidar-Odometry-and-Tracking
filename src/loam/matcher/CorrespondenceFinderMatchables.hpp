#pragma once

#include <srrg_slam_interfaces/correspondence_finder.h>
#include "../features/CustomMatchable.hpp"

namespace Loam{

  //! @brief base class for correspondence finder based on matchables
  //!        estimate is a 3D isometry, and moving and fixed are vectors of visual matchables
  class CorrespondenceFinderMatchables
    : public srrg2_slam_interfaces::CorrespondenceFinder_<srrg2_core::Isometry3f,
                                                          CustomMatchablefVector,
                                                          CustomMatchablefVector> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    struct Stats {
      size_t non_associated    = 0;
      size_t associated_points = 0;
      size_t associated_lines  = 0;
      size_t associated_planes = 0;

      void reset() {
        non_associated    = 0;
        associated_points = 0;
        associated_lines  = 0;
        associated_planes = 0;
      }
    };

    // ia just in case
    using BaseType =
      srrg2_slam_interfaces::CorrespondenceFinder_<srrg2_core::Isometry3f,
                                                   CustomMatchablefVector,
                                                   CustomMatchablefVector>;
    using ThisType = CorrespondenceFinderMatchables;

    //! @brief ctor [everything is static, does nothin]
    CorrespondenceFinderMatchables();

    //! @brief dtor [everything is static, does nothin]
    virtual ~CorrespondenceFinderMatchables();

    void setEstimate(const srrg2_core::Isometry3f& estimate_) override {
      _estimate = estimate_;
    }

    virtual void reset() {
      _stats.reset();
    }

    void compute() override {
    }

    const Stats& stats() const {
      return _stats;
    }

  protected:
    srrg2_core::Isometry3f _estimate = srrg2_core::Isometry3f::Identity();

    //! @brief logging
    Stats _stats;
  };

  //! @brief logging
  std::ostream& operator<<(std::ostream& stream_,
                           const CorrespondenceFinderMatchables::Stats& stats_);

}
