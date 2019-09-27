#pragma once
#include <srrg_messages/instances.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>

namespace Loam{

  class Visualizer {
    public:

      static void visualizeCloud( srrg2_core::ViewerCanvasPtr canvas,const srrg2_core::PointNormalColor3fVectorCloud & t_cloud, const float t_points_size);

  };

}

