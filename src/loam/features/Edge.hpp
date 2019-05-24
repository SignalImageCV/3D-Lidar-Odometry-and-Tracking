#include "../interfaces/FeatureInterface.hpp"

namespace Loam{

   class Edge: FeatureInterface{

    public:
      Edge() = default;

      Edge(int nonsi );

      ~Edge() = default;
  };
}

