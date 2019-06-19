#include "DataPoint.hpp"

namespace Loam{

  DataPoint::DataPoint():DataPointInterface(){};

  DataPoint::DataPoint( const int t_index):
    DataPointInterface( t_index)
  {}; 

  DataPoint::DataPoint(
      const int t_index,
      const bool t_isVert,
      const bool t_isTaken
      ):
    DataPointInterface( t_index,t_isVert,t_isTaken)
  {};
}



