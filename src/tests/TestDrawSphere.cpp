#include "../loam/Visualizer.hpp"

using namespace srrg2_core;
using namespace srrg2_qgl_viewport;
using namespace Loam;

int main( int argc, char** argv){

  string filename = "/home/dinies/temp/trial/tuttty.boss";
  QApplication qapp(argc, argv);
  ViewerCoreSharedQGL viewer(argc, argv, &qapp);
  ViewerCanvasPtr canvas = viewer.getCanvas("drawingSphere");
  std::thread processing_thread(Visualizer::visualizeSphere, canvas, filename);
  viewer.startViewerServer();
  processing_thread.join();
  return 0;
}




