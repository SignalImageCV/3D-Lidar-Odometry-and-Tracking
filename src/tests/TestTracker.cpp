#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_messages/instances.h>
#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg_pcl/point_unprojector_types.h>

#include "loam/tracker/Tracker.hpp"

//good params to show the traking: -vr 64 -hr 1768 -et 7 -er 0.15 -dd 2. -nn 6 -ec 12 -ed 1.5 -en 0.1  -el 0.09 -edl 0.2 -ep 0.02 -edp 0.3


using namespace srrg2_core;
using namespace srrg2_core_ros;
using namespace Loam;
using namespace srrg2_qgl_viewport;



const char* banner[] = {
    "dynamic executor",
      0
};

void processVisualizeTraking(
    ViewerCanvasPtr canvas_1,
    ViewerCanvasPtr canvas_2,
    ViewerCanvasPtr canvas_3,
    const  string & filename,
    const sphericalImage_params t_params);



int main( int argc, char** argv){
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



  messages_registerTypes();
  SE3_registerTypes();
  loam_registerTypes();
  srrgInit( argc, argv, "hi");

  QApplication qapp(argc, argv);
  ViewerCoreSharedQGL viewer(argc, argv, &qapp);
  ViewerCanvasPtr canvas1 = viewer.getCanvas("globalMap");
  ViewerCanvasPtr canvas2 = viewer.getCanvas("currentClusters");
  ViewerCanvasPtr canvas3 = viewer.getCanvas("robot");
  std::thread processing_thread1(
      processVisualizeTraking,
      canvas1,
      canvas2,
      canvas3,
      dataset.value(),
      params
      );

  viewer.startViewerServer();
  processing_thread1.join();
  return 0;
 
}

void processVisualizeTraking(
    ViewerCanvasPtr canvas_1,
    ViewerCanvasPtr canvas_2,
    ViewerCanvasPtr canvas_3,
    const  string & filename,
    const sphericalImage_params t_params){

  PointNormalColor3f origin;
  origin.coordinates() = Vector3f(0.,0.,0.);
  PointNormalColor3fVectorCloud  robotWorldPoints = PointNormalColor3fVectorCloud();


  Tracker tracker( filename, t_params);

  int relative_counter= 0;
  int total_counter= 0;
  const int total_num_iterations = 450;
  const int starting_data_point_index = 0;

  const float points_size = 2.0;

  for( int i= 0; i< starting_data_point_index; ++i){
    tracker.jumpDataEntry();
    ++total_counter;
  }


  while( ViewerCoreSharedQGL::isRunning() and relative_counter < total_num_iterations){
    ++relative_counter;
    ++total_counter;
    tracker.executeCycle();
//   std::cout<< "Iteration number: "<< total_counter<< "\n";
//    std::cout<< "Isometry solution: \n";
//    std::cout << FG_GREEN(tracker.getRelativeT().matrix()) << std::endl;

    PointNormalColor3fVectorCloud robot_world_position_vec;
    robot_world_position_vec.push_back( origin);
    const Isometry3f  world_T= tracker.getAbsoluteT();

    robot_world_position_vec.transformInPlace(world_T);
    for ( auto & p : robot_world_position_vec){
      robotWorldPoints.push_back( p);
    }


    canvas_1->pushPointSize();
    canvas_1->setPointSize(points_size);
    canvas_1->putPoints( tracker.getWorldPoints() );
    canvas_1->popAttribute();
    canvas_1->pushPointSize();
    canvas_1->setPointSize(points_size*3);
    canvas_1->putPoints( robotWorldPoints );
    canvas_1->popAttribute();
    canvas_1->flush();
 
    canvas_2->pushPointSize();
    canvas_2->setPointSize( points_size );
    canvas_2->putPoints(*tracker.m_current_clusterPointsPtr);
    canvas_2->popAttribute();
    canvas_2->flush();

    canvas_3->pushPointSize();
    canvas_3->setPointSize(points_size*3);
    canvas_3->putPoints( robotWorldPoints );
    canvas_3->popAttribute();
    canvas_3->flush();

  }

}


