#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_messages/instances.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_messages/instances.h>

#include "loam/CustomMeasurementAdaptor.hpp"
#include "loam/instances.h"
#include "loam/matcher/CorrespondenceFinderMatchablesKDtree.hpp"
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
  loam_registerTypes();
  srrgInit( argc, argv, "hi");


  DatasetManager dM(  dataset.value());
  BaseSensorMessagePtr cloudPtr_1 = dM.readPointerToMessageFromDataset();
  BaseSensorMessagePtr cloudPtr_2 = dM.readPointerToMessageFromDataset();


  CustomMatchablefVectorData  matchables_1;
  CustomMatchablefVectorData  matchables_2;

  CustomMeasurementAdaptorPtr measurementAdaptor =
    CustomMeasurementAdaptorPtr(new CustomMeasurementAdaptor);

  measurementAdaptor->setMyParams( params);

  measurementAdaptor->setDest( &matchables_1);
  measurementAdaptor->setMeasurement( cloudPtr_1 );
  measurementAdaptor->compute();
  PointNormalColor3fVectorCloud  features_cloud_1 = measurementAdaptor->drawMatchables();
  measurementAdaptor->reset();





  measurementAdaptor->setDest( &matchables_2);
  measurementAdaptor->setMeasurement( cloudPtr_2 );
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

  std::cout << "correspondances [00 - 00] -> " << correspondenceFinder->stats() << std::endl;
  int counter= 0;
  for ( auto & corresp: correspondances){
    ++counter;
    std::cout << " Correspondance num :  " << counter << " fixed index " << corresp.fixed_idx <<" moving index "<< corresp.moving_idx<< std::endl;
    std::cout << " fixed origin               :  " << matchables_1[corresp.fixed_idx].origin().transpose() << " || ";
    std::cout << " fixed direction            :  " << matchables_1[corresp.fixed_idx].direction().transpose() << "\n";
    std::cout << " moving origin              :  " << matchables_2[corresp.moving_idx].origin().transpose() << " || ";
    std::cout << " moving direction           :  " << matchables_2[corresp.moving_idx].direction().transpose() << "\n";
  }
  counter= 0;
  for ( auto & m: matchables_1){
    ++counter;
        std::cout << "Fixed Matchable num  :  " << counter << " ||  orign      :  " << m.origin().transpose()<< std::endl;
        std::cout << "Fixed Matchable num  :  " << counter << " ||  direction  :  " << m.direction().transpose()<< std::endl;
  }
  counter= 0;
  for ( auto & m: matchables_2){
    ++counter;
         std::cout <<"Moving Matchable num :  " << counter << " ||  orign      :  " << m.origin().transpose()<< std::endl;
         std::cout <<"Moving Matchable num :  " << counter << " ||  direction  :  " << m.direction().transpose()<< std::endl;
  }

  std::cout <<"features point number 1:  " <<features_cloud_1.size() << "\n";
  std::cout <<"features point number 2:  " <<features_cloud_2.size() << "\n";
  const float points_size = 2.0;
  QApplication qapp(argc, argv);
  ViewerCoreSharedQGL viewer(argc, argv, &qapp);
  ViewerCanvasPtr canvas1 = viewer.getCanvas("first_scene");
  std::thread processing_thread1( Visualizer::visualizeCloud, canvas1, features_cloud_1, points_size);

  ViewerCanvasPtr canvas2 = viewer.getCanvas("second_scene");
  std::thread processing_thread2( Visualizer::visualizeCloud, canvas2, features_cloud_2, points_size);

  viewer.startViewerServer();
  processing_thread1.join();
  processing_thread2.join();
  

  return 0;
}


