#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_messages/instances.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg_messages/instances.h>


#include <srrg_system_utils/parse_command_line.h>
#include "loam/CustomMeasurementAdaptor.hpp"
#include "loam/instances.h"
#include "loam/matcher/CorrespondenceFinderMatchablesKDtree.hpp"
#include "loam/aligner/instances.h"
#include "loam/DatasetManager.hpp"


using namespace srrg2_core;
using namespace srrg2_solver;
using namespace srrg2_core_ros;
using namespace srrg2_qgl_viewport;
using namespace Loam;


const char* banner[] = {
    "dynamic executor",
      0
};

/* This function generates a fake relative 3d isometry, taken from srrg2_solver tests*/
Isometry3f randomRelativeIso() {
  const float tmax = 0.1;
  float x = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / tmax));
  float y = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / tmax));
  float z = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / tmax));
  float rx = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / tmax));
  float ry = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / tmax));
  float rz = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / tmax));
  Vector6f rand_pose;
  rand_pose << x, y, z, rx, ry, rz;
  Isometry3f rand_iso = srrg2_core::geometry3d::v2t(rand_pose);

  return rand_iso;
}



int main( int argc, char** argv){
  ParseCommandLine cmd_line(argv,banner);
  ArgumentString  dataset (&cmd_line, "d", "dataset", "path to dataset" , "");
  cmd_line.parse();

  messages_registerTypes();
  loam_registerTypes();
  loam_aligner_registerTypes();
  SE3_registerTypes();

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

    CustomAlignerSlice3dPtr aligner =
      CustomAlignerSlice3dPtr( new CustomAlignerSlice3d);

    const Isometry3f T = Isometry3f::Identity() * randomRelativeIso();
    for ( auto & m: matchables_copy){
      m.transformInPlace(T);
    }



    // For now suspended !
    //PointCloud_<std::vector<CustomMatchablef>> matchables_dd;
    //aligner->setFixed(matchables);
    //aligner->setMoving(matchables_copy);
    //aligner->setEstimate(Isometry3f::Identity());

    //aligner->compute();

    //const auto& estimated_T = aligner.estimate();
    //std::cerr << "GT\n" << FG_GREEN(T.matrix()) << std::endl;
    //std::cerr << "estimated\n" << FG_YELLOW(estimated_T.matrix()) << std::endl;
  }

  return 0;
}



 
