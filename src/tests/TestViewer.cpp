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
  int counter = 0;

  while( (msg=src.getMessage()) && ViewerCoreSharedQGL::isRunning()){

    //THis does not work for some unforeseable reasons, leave qgl viewer alone for now !!
   
    cerr<<"there1\n";
    PointCloud2Message* cloud = dynamic_cast<PointCloud2Message*>(msg.get());

    if (cloud) {
      Point3fVectorCloud current_point_cloud;
      cerr<<"there2\n";
      cloud->getPointCloud(current_point_cloud);
      cerr<<"there3\n";
 
      canvas->pushColor();
      canvas->setColor( Vector3f(1.f,5.f,5.f));
      canvas->pushPointSize();
      canvas->setPointSize(1.0);
      canvas->putPoints( current_point_cloud );
      canvas->popAttribute();
      canvas->popAttribute();
      canvas->flush();
      cout << counter<< "\n";
      ++counter;
    }
  }

}


int main( int argc, char** argv){

  ViewerCoreSharedQGL viewer(argc, argv);
  ViewerCanvasPtr canvas = viewer.getCanvas("viewer_test");
  std::thread processing_thread(drawingSubrutine, canvas, std::string(argv[1]));
  viewer.startViewerServer();
  processing_thread.join();

  return 0;
}

