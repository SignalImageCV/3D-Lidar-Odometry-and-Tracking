#include <srrg_system_utils/system_utils.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg_messages/instances.h>
#include "loam/DatasetManager.hpp"



using namespace srrg2_core;
using namespace srrg2_qgl_viewport;
using namespace Loam;


void visualizeSubrutine(ViewerCanvasPtr canvas, const std::string& filename);

int main( int argc, char** argv){

  messages_registerTypes();
  QApplication qapp(argc, argv);
  ViewerCoreSharedQGL viewer(argc, argv, &qapp);
  ViewerCanvasPtr canvas = viewer.getCanvas("viewer_test");
  std::thread processing_thread( visualizeSubrutine, canvas, std::string(argv[1]));
  viewer.startViewerServer();
  processing_thread.join();

  return 0;
}

void visualizeSubrutine(ViewerCanvasPtr canvas, const std::string& filename){

  DatasetManager dM( filename);
  while( ViewerCoreSharedQGL::isRunning()){
    PointNormalColor3fVectorCloud point_cloud = dM.readMessageFromDataset();
    if (point_cloud.size()>0) {
      canvas->pushPointSize();
      canvas->setPointSize(.5);
      canvas->putPoints( point_cloud );
      canvas->flush();
    }
  }
}



