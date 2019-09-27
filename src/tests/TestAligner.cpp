#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_messages/instances.h>
#include <srrg_messages/instances.h>


#include "loam/DatasetManager.hpp"
#include "loam/aligner/Aligner.hpp"


using namespace srrg2_core;
using namespace srrg2_core_ros;
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
  SE3_registerTypes();
  loam_registerTypes();

  srrgInit( argc, argv, "hi");

  DatasetManager dM(  dataset.value());
  BaseSensorMessagePtr cloudPtr = dM.readPointerToMessageFromDataset();

  Aligner aligner = Aligner();

  while( cloudPtr ) {

    Isometry3f solution =   aligner.compute(cloudPtr);
    std::cout<< "Isometry solution: \n";
    std::cout << FG_GREEN(solution.matrix()) << std::endl;

    cloudPtr = dM.readPointerToMessageFromDataset();
  }

  return 0;
}


