#include "../loam/Visualizer.hpp"
#include <srrg_system_utils/parse_command_line.h>
#include "../loam/CustomMeasurementAdaptor.hpp"
#include "../loam/instances.h"



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
    CustomMeasurementAdaptorPtr measurementAdaptor = CustomMeasurementAdaptorPtr(new CustomMeasurementAdaptor);
    measurementAdaptor->setDest( &matchables);
    measurementAdaptor->setMeasurement( cloudPtr );
    measurementAdaptor->compute();
    std::cout << "final  matchables size : "<< matchables.size()<< "\n";


    cloudPtr = dM.readPointerToMessageFromDataset();
  }
  return 0;
}

