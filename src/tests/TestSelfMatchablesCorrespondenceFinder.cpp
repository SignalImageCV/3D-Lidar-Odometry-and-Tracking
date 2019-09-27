#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_messages/instances.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg_messages/instances.h>


#include "loam/CustomMeasurementAdaptor.hpp"
#include "loam/instances.h"
#include "loam/matcher/CorrespondenceFinderMatchablesKDtree.hpp"
#include "loam/DatasetManager.hpp"


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

    const float x = 0.;
    const float y = 0.3;
    const float z = 0.3;
    const float rx = 0;
    const float ry = 0;
    const float rz = 0;
    Vector6f pose;
    pose << x, y, z, rx, ry, rz;
    const Isometry3f rotoTransl = srrg2_core::geometry3d::v2t(pose);

    const Isometry3f T = Isometry3f::Identity() * rotoTransl;
    for ( auto & m: matchables_copy){
      m.transformInPlace(T);
    }


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
      std::cout << " fixed origin               :  " << matchables[corresp.fixed_idx].origin().transpose() << " || ";
      std::cout << " fixed direction            :  " << matchables[corresp.fixed_idx].direction().transpose() << "\n";
      const CustomMatchablef  m_transf =  matchables[corresp.fixed_idx].transform(T); 
      std::cout << " fixed origin trasformed    :  " << m_transf.origin().transpose() << " || ";
      std::cout << " fixed direction transformed:  " << m_transf.direction().transpose() << "\n";
      std::cout << " moving origin              :  " << matchables_copy[corresp.moving_idx].origin().transpose() << " || ";
      std::cout << " moving direction           :  " << matchables_copy[corresp.moving_idx].direction().transpose() << "\n";
    }
    counter= 0;
   for ( auto & match: matchables){
     ++counter;
 //    std::cout << "Fixed Matchable num  :  " << counter << " ||  orign      :  " << match.origin().transpose()<< std::endl;
 //    std::cout << "Fixed Matchable num  :  " << counter << " ||  direction  :  " << match.direction().transpose()<< std::endl;
   }
    counter= 0;
    for ( auto & match_copy: matchables_copy){
      ++counter;
 //     std::cout <<"Moving Matchable num :  " << counter << " ||  orign      :  " << match_copy.origin().transpose()<< std::endl;
 //     std::cout <<"Moving Matchable num :  " << counter << " ||  direction  :  " << match_copy.direction().transpose()<< std::endl;
    }


    cloudPtr = dM.readPointerToMessageFromDataset();
  }
  return 0;
}


