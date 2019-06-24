#include "../loam/Visualizer.hpp"

using namespace srrg2_core;
using namespace srrg2_qgl_viewport;
using namespace Loam;


int main( int argc, char** argv){

  string filename = "/home/dinies/temp/trial/tuttty.boss";
  ViewerCoreSharedQGL viewer(argc, argv);
  ViewerCanvasPtr canvas = viewer.getCanvas("PointsWithNormals");
  std::thread processing_thread(
      Visualizer::visualizePointsWithNormals,canvas,filename);
  viewer.startViewerServer();
  processing_thread.join();
  return 0;
}



