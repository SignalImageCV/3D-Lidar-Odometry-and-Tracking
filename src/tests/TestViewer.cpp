#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>
#include <thread>
#include "../loam/DatasetManager.hpp"
using namespace std;
using namespace srrg2_core;
using namespace srrg2_qgl_viewport;
using namespace Loam;


void drawingSubrutine(ViewerCanvasPtr canvas, const std::string& filename){

  MessageFileSource src;
  src.open(filename);

  BaseSensorMessagePtr msg;

  while( (msg=src.getMessage()) && ViewerCoreSharedQGL::isRunning()){

   
    PointCloud2Message* cloud = dynamic_cast<PointCloud2Message*>(msg.get());

    if (cloud) {
      Point3fVectorCloud current_point_cloud;
      cloud->getPointCloud(current_point_cloud);
 
      canvas->pushColor();
      canvas->setColor( Vector3f(0.f,0.f,0.f));
      canvas->pushPointSize();
      canvas->setPointSize(1.0);
      canvas->putPoints( current_point_cloud );
      canvas->popAttribute();
      canvas->popAttribute();
      canvas->flush();
    }
  }

}


int main( int argc, char** argv){

  messages_registerTypes();
  ViewerCoreSharedQGL viewer(argc, argv);
  ViewerCanvasPtr canvas = viewer.getCanvas("viewer_test");
  std::thread processing_thread(drawingSubrutine, canvas, std::string(argv[1]));
  viewer.startViewerServer();
  processing_thread.join();

  return 0;
}

