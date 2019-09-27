#include <srrg_system_utils/system_utils.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg_messages/instances.h>
#include "loam/Drawer.hpp"



using namespace srrg2_core;
using namespace srrg2_qgl_viewport;
using namespace Loam;

void visualizeAxes( ViewerCanvasPtr canvas);
void drawAxes(ViewerCanvasPtr canvas, const vector<PointNormalColor3fVectorCloud> & t_axes);

int main( int argc, char** argv){

  QApplication qapp(argc, argv);
  ViewerCoreSharedQGL viewer(argc, argv, &qapp);
  ViewerCanvasPtr canvas = viewer.getCanvas("drawingPoints_test");
  std::thread processing_thread( visualizeAxes, canvas);
  viewer.startViewerServer();
  processing_thread.join();

  return 0;
}

void visualizeAxes(ViewerCanvasPtr canvas ){

  vector<PointNormalColor3fVectorCloud> axes =  Drawer::createAxes();
  while(ViewerCoreSharedQGL::isRunning()){
    canvas->pushPointSize();
    canvas->setPointSize(4.0);
    drawAxes( canvas, axes);
    canvas->flush();
  }
}


void drawAxes(ViewerCanvasPtr canvas, const vector<PointNormalColor3fVectorCloud> & t_axes){
  for( auto & axis: t_axes){
    canvas->putPoints( axis);
  }
}


