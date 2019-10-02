#include "Map.hpp"

namespace Loam{

  Map::Map(){
      m_worldPointsPtr= std::make_shared<PointNormalColor3fVectorCloud>();
  };

  Map::~Map(){};

  void Map::insertPoints( PointNormalColor3fVectorCloud &  t_current_points, const Isometry3f & t_transformation){

    t_current_points.transformInPlace( t_transformation);

    m_worldPointsPtr->insert(
      m_worldPointsPtr->end(),
      std::make_move_iterator( t_current_points.begin()),
      std::make_move_iterator( t_current_points.end())
    );
  }
}
