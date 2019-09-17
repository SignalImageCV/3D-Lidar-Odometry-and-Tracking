#include "../loam/Visualizer.hpp"
#include <srrg_system_utils/parse_command_line.h>

using namespace srrg2_core;
using namespace srrg2_core_ros;
using namespace srrg2_qgl_viewport;
using namespace Loam;

const char* banner[] = {
    "dynamic executor",
      0
};


int main( int argc, char** argv){

  ParseCommandLine cmd_line(argv,banner);
  ArgumentString  dataset (&cmd_line, "d", "dataset", "path to dataset" , "");
  cmd_line.parse();

  messages_registerTypes();
  srrgInit( argc, argv, "hi");


  QApplication qapp(argc, argv);
  ViewerCoreSharedQGL viewer(argc, argv, &qapp);
  ViewerCanvasPtr canvas = viewer.getCanvas("drawingCloud");
  std::thread processing_thread(Visualizer::visualizeCloudSmoothness, canvas, dataset.value());
  viewer.startViewerServer();
  processing_thread.join();
  return 0;
}




