#include "./CorrespondenceFinderMatchablesBruteForce.hpp"

namespace Loam{

  using namespace srrg2_core;

  void CorrespondenceFinderMatchablesBruteForce::reset() {
    BaseType::reset();

    _is_initialized = false;

    if (!_fixed) {
      throw std::runtime_error("CorrespondenceFinderMatchablesKDTree::reset|error, fixed not set");
    }

    //create matrix associations
    
  }

  void CorrespondenceFinderMatchablesBruteForce::compute() {
    _correspondences->reserve(_moving->size());


    //find good associations
  }
}
