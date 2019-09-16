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

void visualizeClouder( ViewerCanvasPtr canvas, string filename){
  DatasetManager dM( filename);
  const sphericalImage_params params(
    64, //num_vertical_rings
    768, //num_points_ring
    7, //epsilon_times
    0.15, //epsilon_radius
    1, //depth_differential_threshold
    4,  //min_neighboors_for_normal
    8, //epsilon_c
    0.1, //epsilon_d
    0.02, //epsilon_n
    1, //epsilon_l
    1, //epsilon_dl
    1, //epsilon_p
    1 //epsilon_dp
  );
 
  SphericalDepthImage sph_Image;
  while( ViewerCoreSharedQGL::isRunning()){

    PointNormalColor3fVectorCloud cloud = dM.readMessageFromDataset();

    sph_Image = SphericalDepthImage(cloud,params);
    sph_Image.initializeIndexImage();
    sph_Image.removeFlatSurfaces();
    sph_Image.executeOperations();

    const PointNormalColor3fVectorCloud cloud_final = sph_Image.getPointCloud();
  
    canvas->pushPointSize();
    canvas->setPointSize(1.5f);
    canvas->putPoints( cloud_final);
    canvas->popAttribute();
    canvas->flush();
  }
}

int main( int argc, char** argv){
  ParseCommandLine cmd_line(argv,banner);
  ArgumentString  dataset (&cmd_line, "d", "dataset", "path to dataset" , "");
  cmd_line.parse();

  messages_registerTypes();
  srrgInit( argc, argv, "hi");
 


  QApplication qapp(argc, argv);
  ViewerCoreSharedQGL viewer(argc, argv, &qapp);
  ViewerCanvasPtr canvas1 = viewer.getCanvas("cloud");

  std::thread processing_thread(
      visualizeClouder,canvas1, dataset.value());

  viewer.startViewerServer();
  processing_thread.join();
  return 0;
}
