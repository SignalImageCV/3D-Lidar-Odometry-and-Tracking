#include "Visualizer.hpp"

using namespace srrg2_core;
using namespace srrg2_qgl_viewport;
namespace Loam{
  void Visualizer::visualizeCloud( ViewerCanvasPtr canvas,const PointNormalColor3fVectorCloud & t_cloud, const float t_points_size){
    while( ViewerCoreSharedQGL::isRunning()){
        canvas->pushPointSize();
        canvas->setPointSize(t_points_size);
        canvas->putPoints( t_cloud);
        canvas->flush();
    }
  }
}
