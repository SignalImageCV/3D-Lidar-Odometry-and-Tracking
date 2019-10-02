#pragma once
#include <srrg_system_utils/system_utils.h>
#include <srrg_pcl/point_unprojector_types.h>
#include <srrg_messages/instances.h>
#include <srrg_messages_ros/message_handlers/message_rosbag_source.h>


namespace Loam{
  using namespace srrg2_core;

  class Map{
    public:
      std::shared_ptr<PointNormalColor3fVectorCloud> m_worldPointsPtr;
      Map();
      virtual ~Map();

      inline const  PointNormalColor3fVectorCloud getWorldPoints() const { return *m_worldPointsPtr; };

      void insertPoints( PointNormalColor3fVectorCloud &  t_current_points, const Isometry3f & t_transformation);

  };

}

