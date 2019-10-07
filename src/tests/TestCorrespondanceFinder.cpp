#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_messages/instances.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_messages/instances.h>

#include "loam/CustomMeasurementAdaptor.hpp"
#include "loam/instances.h"
#include "loam/matcher/CorrespondenceFinderMatchablesKDtree.hpp"
#include "loam/matcher/CorrespondenceFinderMatchablesBruteForce.hpp"
#include "loam/DatasetManager.hpp"
#include "loam/Visualizer.hpp"


using namespace srrg2_core;
using namespace srrg2_core_ros;
using namespace srrg2_qgl_viewport;
using namespace Loam;


const char* banner[] = {
    "dynamic executor",
      0
};

void processCorrespondences_test(
    ViewerCanvasPtr canvas_1,
    ViewerCanvasPtr canvas_2,
    ViewerCanvasPtr canvas_3,
    const  string & filename,
    const sphericalImage_params t_params);

PointNormalColor3fVectorCloud drawLine( Vector3f p1,  Vector3f p2, Vector3f color );



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
  ViewerCanvasPtr canvas1 = viewer.getCanvas("old_matchables");
  ViewerCanvasPtr canvas2 = viewer.getCanvas("new_matchables");
  ViewerCanvasPtr canvas3 = viewer.getCanvas("correspondances");
  std::thread processing_thread1(
      processCorrespondences_test,
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

PointNormalColor3fVectorCloud drawLine( Vector3f p1,  Vector3f p2, Vector3f color ){

  const float dist_points = (p1 - p2).norm();
  const float epsilon = 0.1;
  const int num_points = static_cast <int> ( dist_points / epsilon);
  PointNormalColor3fVectorCloud linePoints;
  linePoints.resize( num_points);

  const Vector3f line_vec= p2 - p1; 
  for ( float  i = 0; i< num_points; ++i){
    const float curr_advancement= ( i / num_points);
    const Vector3f from_p1 =  line_vec * curr_advancement; 
    const Vector3f curr_coord = p1 + from_p1;
    linePoints[i].coordinates() = curr_coord;
    linePoints[i].color() = color;
 }
  return linePoints;
}

void processCorrespondences_test(
    ViewerCanvasPtr canvas_1,
    ViewerCanvasPtr canvas_2,
    ViewerCanvasPtr canvas_3,
    const  string & filename,
    const sphericalImage_params t_params){


  DatasetManager dM(  filename);
  int relative_counter= 0;
  int total_counter= 0;
  const int total_num_iterations = 200;
  const int starting_data_point_index = 0;

  BaseSensorMessagePtr old_cloudPtr;
  BaseSensorMessagePtr new_cloudPtr;

  for( int i= 0; i< starting_data_point_index; ++i){
    dM.readPointerToMessageFromDataset();
    std::cout <<"Iteration num : " << total_counter<<"\n"; 
    ++total_counter;
  }


  while (  ViewerCoreSharedQGL::isRunning() and relative_counter < total_num_iterations){
    ++relative_counter;
    ++total_counter;
    new_cloudPtr = dM.readPointerToMessageFromDataset();
    CustomMatchablefVectorData  matchables_1;
    CustomMatchablefVectorData  matchables_2;

    CustomMeasurementAdaptorPtr measurementAdaptor =
      CustomMeasurementAdaptorPtr(new CustomMeasurementAdaptor);

    measurementAdaptor->setMyParams( t_params);

    measurementAdaptor->setDest( &matchables_1);
    measurementAdaptor->setMeasurement( old_cloudPtr);
    measurementAdaptor->compute();
    PointNormalColor3fVectorCloud  features_cloud_1 = measurementAdaptor->drawMatchables();
    measurementAdaptor->reset();

    measurementAdaptor->setDest( &matchables_2);
    measurementAdaptor->setMeasurement( new_cloudPtr);
    measurementAdaptor->compute();
    PointNormalColor3fVectorCloud  features_cloud_2 = measurementAdaptor->drawMatchables();


    CorrespondenceVector correspondances;


    CorrespondenceFinderMatchablesKDTreePtr correspondenceFinder =
     CorrespondenceFinderMatchablesKDTreePtr( new CorrespondenceFinderMatchablesKDTree);


    correspondenceFinder->setCorrespondences( &correspondances);
    correspondenceFinder->setEstimate(Isometry3f::Identity() );
    correspondenceFinder->setFixed(&matchables_1 );
    correspondenceFinder->setMoving(&matchables_2);
    correspondenceFinder->reset();
    correspondenceFinder->compute();

    old_cloudPtr = new_cloudPtr;


    std::cout << "iteration number : "   <<total_counter<< std::endl;
    std::cout << "correspondances || " << correspondenceFinder->stats() << std::endl;
    int counter= 0;
    for ( auto & corresp: correspondances){
      ++counter;
  //    std::cout << " Correspondance num :  " << counter << " fixed index " << corresp.fixed_idx <<" moving index "<< corresp.moving_idx<< std::endl;
//      std::cout << " fixed origin               :  " << matchables_1[corresp.fixed_idx].origin().transpose() << " || ";
//      std::cout << " fixed direction            :  " << matchables_1[corresp.fixed_idx].direction().transpose() << "\n";
//      std::cout << " moving origin              :  " << matchables_2[corresp.moving_idx].origin().transpose() << " || ";
//      std::cout << " moving direction           :  " << matchables_2[corresp.moving_idx].direction().transpose() << "\n";
    }
    counter= 0;
    for ( auto & m: matchables_1){
      ++counter;
      //        std::cout << "Fixed Matchable num  :  " << counter << " ||  orign      :  " << m.origin().transpose()<< std::endl;
      //        std::cout << "Fixed Matchable num  :  " << counter << " ||  direction  :  " << m.direction().transpose()<< std::endl;
    }
    counter= 0;
    for ( auto & m: matchables_2){
      ++counter;
      //         std::cout <<"Moving Matchable num :  " << counter << " ||  orign      :  " << m.origin().transpose()<< std::endl;
      //         std::cout <<"Moving Matchable num :  " << counter << " ||  direction  :  " << m.direction().transpose()<< std::endl;
    }

    //std::cout <<"features point number 1:  " <<features_cloud_1.size() << "\n";
    //std::cout <<"features point number 2:  " <<features_cloud_2.size() << "\n";
    const float points_size = 1.5;
    canvas_1->pushPointSize();
    canvas_1->setPointSize(points_size);
    canvas_1->putPoints( features_cloud_1);
    canvas_1->flush();
 
    canvas_2->pushPointSize();
    canvas_2->setPointSize(points_size);
    canvas_2->putPoints( features_cloud_2);
    canvas_2->flush();

    canvas_3->pushPointSize();
    canvas_3->setPointSize(points_size);
    canvas_3->putPoints( features_cloud_1);
    canvas_3->putPoints( features_cloud_2);
    canvas_3->popAttribute();
    canvas_3->setPointSize(points_size*4);

    for ( auto & c: correspondances){
      const PointNormalColor3fVectorCloud  linePoints =
        drawLine(
            matchables_1[c.fixed_idx].origin(),
            matchables_2[c.moving_idx].origin(),
            Vector3f(0.,0.,0. ));
      canvas_3->putPoints( linePoints);
      
    }
    canvas_3->popAttribute();
    canvas_3->flush();
  }
}


