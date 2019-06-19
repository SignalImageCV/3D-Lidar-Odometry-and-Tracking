#pragma once
#include "../interfaces/DataPointInterface.hpp"

namespace Loam{


  class DataPoint: public DataPointInterface{
    public:

      DataPoint();

      DataPoint( const int t_index);

      DataPoint(
          const int t_index,
          const bool t_isVert,
          const bool t_isTaken
          );
      
  };
}
  



