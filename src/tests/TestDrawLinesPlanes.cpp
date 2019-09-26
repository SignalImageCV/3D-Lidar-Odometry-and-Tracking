#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>
#include <thread>

#include "../loam/Drawer.hpp"


using namespace srrg2_core;
using namespace srrg2_qgl_viewport;
using namespace Loam;

void visualizeLinesPlanes(ViewerCanvasPtr canvas);

int main( int argc, char** argv){

  QApplication qapp(argc, argv);
  ViewerCoreSharedQGL viewer(argc, argv, &qapp);
  ViewerCanvasPtr canvas = viewer.getCanvas("drawingLinesAndPlanes");
  std::thread processing_thread( visualizeLinesPlanes, canvas);
  viewer.startViewerServer();
  processing_thread.join();

  return 0;
}
void visualizeLinesPlanes(ViewerCanvasPtr canvas){

    PointNormalColor3fVectorCloud l1 = Drawer::createLine(
        Vector3f( 7.,7.,7.), Vector3f( 0.,-1.,0.), 10, 0.01);
    PointNormalColor3fVectorCloud l2 = Drawer::createLine(
        Vector3f( -7.,7.,-7.), Vector3f( 0.,1.,1.), 30, 0.01);
    PointNormalColor3fVectorCloud p1 = Drawer::createPlane(
        Vector3f( 5.,5.,0.), Vector3f( 0.,0.,1.), Vector3f( 1.,-1.,0.).normalized(),
        2, 3, 0.1, 0.1);

    while(ViewerCoreSharedQGL::isRunning()){
      canvas->pushPointSize();
      canvas->setPointSize(2.0);
      canvas->putPoints( l1);
      canvas->putPoints( l2);
      canvas->putPoints( p1);
      canvas->flush();
    }
  }


