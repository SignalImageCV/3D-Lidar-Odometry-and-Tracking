#include "Map.hpp"

namespace Loam{

  Map::Map():
    m_max_numPoints( 180000),
    m_worldPointsPtr(std::make_shared<PointNormalColor3fVectorCloud>())
  {
  };

  Map::~Map(){};

  void Map::insertPoints( PointNormalColor3fVectorCloud &  t_current_points, const Isometry3f & t_transformation){

    t_current_points.transformInPlace( t_transformation);

    m_worldPointsPtr->insert(
      m_worldPointsPtr->end(),
      std::make_move_iterator( t_current_points.begin()),
      std::make_move_iterator( t_current_points.end())
    );

    if ( m_worldPointsPtr->size() >  m_max_numPoints){
      int num_points_toRemove = m_worldPointsPtr->size() -  m_max_numPoints;
      m_worldPointsPtr->erase(
          m_worldPointsPtr->begin(),
          m_worldPointsPtr->begin()+ num_points_toRemove);

    }
  }
}
