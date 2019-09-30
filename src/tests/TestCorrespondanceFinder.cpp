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
  cmd_line.parse();

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
    //    std::cout << "Fixed Matchable num  :  " << counter << " ||  orign      :  " << m.origin().transpose()<< std::endl;
    //    std::cout << "Fixed Matchable num  :  " << counter << " ||  direction  :  " << m.direction().transpose()<< std::endl;
  }
  counter= 0;
  for ( auto & m: matchables_2){
    ++counter;
    //     std::cout <<"Moving Matchable num :  " << counter << " ||  orign      :  " << m.origin().transpose()<< std::endl;
    //     std::cout <<"Moving Matchable num :  " << counter << " ||  direction  :  " << m.direction().transpose()<< std::endl;
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


