#include "../interfaces/FeatureInterface.hpp"

namespace Loam{

   class Surface: FeatureInterface{

    public:
      Surface() = default;

      Surface(int nonsi );

      ~Surface() = default;
  };
}

