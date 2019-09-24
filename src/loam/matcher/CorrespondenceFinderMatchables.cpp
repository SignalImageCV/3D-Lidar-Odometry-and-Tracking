#include "./CorrespondenceFinderMatchables.hpp"

#include <iostream>

namespace Loam{

  std::ostream& operator<<(std::ostream& stream_,
                           const CorrespondenceFinderMatchables::Stats& stats_) {
    stream_ << "associations: "
            << stats_.associated_points + stats_.associated_lines + stats_.associated_planes << " ";
    stream_ << "{pn: " << stats_.associated_points << ", ln: " << stats_.associated_lines
            << ", pl: " << stats_.associated_planes << "} ";
    stream_ << "|| unassociated: " << stats_.non_associated;
    return stream_;
  }

  CorrespondenceFinderMatchables::CorrespondenceFinderMatchables() {
    // ia empty, everything is static
  }

  CorrespondenceFinderMatchables::~CorrespondenceFinderMatchables() {
    // ia empty, everything is static
  }
} 
