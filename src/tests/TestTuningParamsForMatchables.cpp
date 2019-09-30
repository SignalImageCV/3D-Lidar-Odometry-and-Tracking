#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_messages/instances.h>
#include <srrg_system_utils/shell_colors.h>

#include "loam/instances.h"
#include "loam/Visualizer.hpp"
#include "loam/DatasetManager.hpp"
#include "loam/Visualizer.hpp"
#include "loam/features/SphericalDepthImage.hpp"



using namespace srrg2_core;
using namespace srrg2_core_ros;
using namespace srrg2_qgl_viewport;
using namespace Loam;

const char* banner[] = {
    "dynamic executor",
      0
};


void processVisualizeRealMatchables(
    ViewerCanvasPtr canvas_1,
    ViewerCanvasPtr canvas_2,
    const  string & filename,
    const sphericalImage_params t_params);


int main( int argc, char** argv){

  messages_registerTypes();
  loam_registerTypes();
  srrgInit( argc, argv, "hi");

  ParseCommandLine cmd_line(argv,banner);
  ArgumentString  dataset (&cmd_line, "d", "dataset", "path to dataset" , "");
  ArgumentString  epsilon_l(&cmd_line, "l", "epsilon_l", "parameter that defines epsilon_l" , "");
  ArgumentString  epsilon_dl(&cmd_line, "dl", "epsilon_dl", "parameter that defines epsilon_dl" , "");
  ArgumentString  epsilon_p(&cmd_line, "p", "epsilon_p", "parameter that defines epsilon_p" , "");
  ArgumentString  epsilon_dp(&cmd_line, "dp", "epsilon_dp", "parameter that defines epsilon_dp" , "");
  cmd_line.parse();

  const sphericalImage_params params(
    64, //num_vertical_rings
    768, //num_points_ring
    7, //epsilon_times
    0.15, //epsilon_radius
    2.1, //depth_differential_threshold
    7,  //min_neighboors_for_normal
    10, //epsilon_c
    1.5, //epsilon_d
    0.3, //epsilon_n
    std::stof( epsilon_l.value()), //epsilon_l
    std::stof( epsilon_dl.value()), //epsilon_dl
    std::stof( epsilon_p.value()), //epsilon_p
    std::stof( epsilon_dp.value())//epsilon_dp
  );

  QApplication qapp(argc, argv);
  ViewerCoreSharedQGL viewer(argc, argv, &qapp);
  ViewerCanvasPtr canvas1 = viewer.getCanvas("clusters");
  ViewerCanvasPtr canvas2 = viewer.getCanvas("matchables");
  std::thread processing_thread1(
      processVisualizeRealMatchables,
      canvas1,
      canvas2,
      dataset.value(),
      params
      );

  viewer.startViewerServer();
  processing_thread1.join();
  return 0;
}


void processVisualizeRealMatchables(
    ViewerCanvasPtr canvas_1,
    ViewerCanvasPtr canvas_2,
    const  string & filename,
    const sphericalImage_params t_params){
  DatasetManager dM(  filename);
  BaseSensorMessagePtr cloudPtr;
  cloudPtr = dM.readPointerToMessageFromDataset();
  while( ViewerCoreSharedQGL::isRunning() and  cloudPtr){

    CustomMatchablefVectorData  matchables;
    CustomMeasurementAdaptorPtr measurementAdaptor =
      CustomMeasurementAdaptorPtr(new CustomMeasurementAdaptor);
    measurementAdaptor->setDest( &matchables);
    measurementAdaptor->setMeasurement( cloudPtr );
    measurementAdaptor->compute();
    PointNormalColor3fVectorCloud  clusters_cloud = measurementAdaptor->drawClusters();
    PointNormalColor3fVectorCloud  matchables_cloud= measurementAdaptor->drawMatchables();

    const float points_size = 2.0;
    Visualizer::visualizeCloud( canvas_1, clusters_cloud, points_size);
    Visualizer::visualizeCloud( canvas_2, matchables_cloud, points_size);
    cloudPtr = dM.readPointerToMessageFromDataset();
  }
}




