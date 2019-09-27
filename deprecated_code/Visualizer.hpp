#pragma once
#include <iostream>
#include <thread>
#include <vector>
#include <list>

#include <srrg_system_utils/system_utils.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg_boss/id_context.h>
#include <srrg_boss/serializer.h>
#include <srrg_boss/deserializer.h>
#include <srrg_messages/instances.h>

#include "DatasetManager.hpp"
#include "MyMath.hpp"
#include "Drawer.hpp"

namespace Loam{
  class Visualizer {
    public:



      static void visualizeCloud( ViewerCanvasPtr canvas,const PointNormalColor3fVectorCloud & t_cloud, const float t_points_size);





  };

}

  //  color3fRed
  //  color3fOrange
  //  color3fYellow
  //  color3fGreen
  //  color3fCyan
  //  color3fMagenta
  //  color3fViolet
  //  color3fBlue
  //  color3fBlack
  //  color3fWhite
  //  color3fDarkRed
  //  color3fDarkYellow
  //  color3fDarkGreen
  //  color3fDarkCyan
  //  color3fDarkMagenta
  //  color3fDarkViolet
  //  color3fDarkBlue
  //  color3fMint
  //  color3fLightGray
  //  color3fEerieBlack
  //  color3fPaleYellow
  //  color3fGunmetal

