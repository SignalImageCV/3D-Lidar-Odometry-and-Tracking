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
  ArgumentString  num_vertical_rings(&cmd_line, "vr", "num_vertical_rings", "num of vertical rings" , "");
  ArgumentString  num_points_ring(&cmd_line, "hr", "num_points_ring", "num of horizontal rings (slices)" , "");
  ArgumentString  epsilon_t(&cmd_line, "et", "epsilon_t", "min num of times points have to fall in the base circle to be considered vertical " , "");
  ArgumentString  epsilon_r(&cmd_line, "er", "epsilon_r", "min radius of the circle in which vertical points have to fall" , "");
  ArgumentString  depth_differential_threshold(&cmd_line, "dd", "depth_differential_threshold", "max difference between depths in near points to be considered valid points" , "");
  ArgumentString  min_neighboors_for_normal(&cmd_line, "nn", "min_neighboors_for_normal", "min num of points to use to compute the normals" , "");
  ArgumentString  epsilon_c(&cmd_line, "ec", "epsilon_c", "min num of points that forma a cluster" , "");
  ArgumentString  epsilon_d(&cmd_line, "ed", "epsilon_d", "min cartesian distance between points of the same cluster" , ""); //currently unused
  ArgumentString  epsilon_n(&cmd_line, "en", "epsilon_n", "min distance between directions of normals of points in the same cluster" , "");
  ArgumentString  epsilon_l(&cmd_line, "el", "epsilon_l", "min number that descibes the eigenvalue constraint of a line" , "");
  ArgumentString  epsilon_dl(&cmd_line,"edl", "epsilon_dl", "min number of the cumulative cartesian error of the matchable fitted to a line" , "");
  ArgumentString  epsilon_p(&cmd_line, "ep", "epsilon_p", "min number that descibes the eigenvalue constraint of a plane" , "");
  ArgumentString  epsilon_dp(&cmd_line,"edp", "epsilon_dp", "min number of the cumulative cartesian error of the matchable fitted to a plane" , "");
  cmd_line.parse();

  const sphericalImage_params params(
    std::stoi( num_vertical_rings.value()), 
    std::stoi( num_points_ring.value()),
    std::stoi( epsilon_t.value()),
    std::stof( epsilon_r.value()),
    std::stof( depth_differential_threshold.value()),
    std::stoi( min_neighboors_for_normal.value()),
    std::stoi( epsilon_c.value()),
    std::stof( epsilon_d.value()),
    std::stof( epsilon_n.value()),
    std::stof( epsilon_l.value()),
    std::stof( epsilon_dl.value()),
    std::stof( epsilon_p.value()),
    std::stof( epsilon_dp.value())
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

  const float points_size = 2.0;
  while( ViewerCoreSharedQGL::isRunning() and  cloudPtr){

    CustomMatchablefVectorData  matchables;
    CustomMeasurementAdaptorPtr measurementAdaptor =
      CustomMeasurementAdaptorPtr(new CustomMeasurementAdaptor);

    measurementAdaptor->setMyParams( t_params);
    measurementAdaptor->setDest( &matchables);
    measurementAdaptor->setMeasurement( cloudPtr );
    measurementAdaptor->compute();
    PointNormalColor3fVectorCloud  clusters_cloud = measurementAdaptor->drawClusters();
    PointNormalColor3fVectorCloud  matchables_cloud= measurementAdaptor->drawMatchables();

    canvas_1->pushPointSize();
    canvas_1->setPointSize(points_size);
    canvas_1->putPoints( clusters_cloud);
    canvas_1->flush();
 
    canvas_2->pushPointSize();
    canvas_2->setPointSize(points_size);
    canvas_2->putPoints( matchables_cloud);
    canvas_2->flush();
 
    cloudPtr = dM.readPointerToMessageFromDataset();
  }
}




