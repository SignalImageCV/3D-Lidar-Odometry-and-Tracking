#include "../loam/Visualizer.hpp"
#include <srrg_system_utils/parse_command_line.h>
#include "../loam/CustomMeasurementAdaptor.hpp"
#include "../loam/instances.h"
#include "../loam/matcher/CorrespondenceFinderMatchablesKDtree.hpp"



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
  BaseSensorMessagePtr cloudPtr = dM.readPointerToMessageFromDataset();

  while( cloudPtr ) {

    CustomMatchablefVectorData  matchables;

    CustomMeasurementAdaptorPtr measurementAdaptor =
      CustomMeasurementAdaptorPtr(new CustomMeasurementAdaptor);
    measurementAdaptor->setDest( &matchables);
    measurementAdaptor->setMeasurement( cloudPtr );
    measurementAdaptor->compute();

    CustomMatchablefVectorData  matchables_copy = matchables;

    CorrespondenceVector correspondances;

    CorrespondenceFinderMatchablesKDTreePtr correspondenceFinder =
      CorrespondenceFinderMatchablesKDTreePtr( new CorrespondenceFinderMatchablesKDTree);

    correspondenceFinder->setCorrespondences( &correspondances);
    correspondenceFinder->setEstimate(Isometry3f::Identity() );
    correspondenceFinder->setFixed(&matchables );
    correspondenceFinder->setMoving(&matchables_copy );
    correspondenceFinder->reset();
    correspondenceFinder->compute();

    std::cout << "correspondances [00 - 00] -> " << correspondenceFinder->stats() << std::endl;
    int counter= 0;
    for ( auto & corresp: correspondances){
      ++counter;
      std::cout << " Correspondance num :  " << counter << " fixed index " << corresp.fixed_idx <<" moving index "<< corresp.moving_idx<< std::endl;
    }
    counter= 0;
 //   for ( auto & match: matchables){
//      ++counter;
//      std::cout << "Fixed Matchable num :  " << counter << " orign :  " << match.origin().transpose()<< std::endl;
//    }
//    counter= 0;
//    for ( auto & match: matchables_copy){
//      ++counter;
//      std::cout << "Moving Matchable num :  " << counter << " orign :  " << match.origin().transpose()<< std::endl;
//    }


    cloudPtr = dM.readPointerToMessageFromDataset();
  }
  return 0;
}


