#include "../loam/Visualizer.hpp"
using namespace srrg2_core;
using namespace srrg2_qgl_viewport;
using namespace Loam;


int main( int argc, char** argv){

  messages_registerTypes();
  ViewerCoreSharedQGL viewer(argc, argv);
  ViewerCanvasPtr canvas = viewer.getCanvas("viewer_test");
  std::thread processing_thread( Visualizer::visualizeSubrutine, canvas, std::string(argv[1]));
  viewer.startViewerServer();
  processing_thread.join();

  return 0;
}

