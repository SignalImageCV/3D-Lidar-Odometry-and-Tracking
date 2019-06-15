#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg_system_utils/shell_colors.h>
#include <thread>

#include "../loam/features/SphericalDepthImage.hpp"
#include "../loam/features/FeatureExtractor.hpp"
#include "../loam/DatasetManager.hpp"

namespace Loam{
  class Visualizer {
    public:

      static void drawingAxes(ViewerCanvasPtr canvas);

      static void visualizeSphere(ViewerCanvasPtr canvas,const string & filename);

      static void visualizeCloud( ViewerCanvasPtr canvas, const  string & filename);



  };

}



