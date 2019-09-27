#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg_messages/instances.h>

#include <srrg_solver/solver_core/instances.h>
#include <srrg_solver/SE3/instances.h>


#include "loam/CustomMeasurementAdaptor.hpp"
#include "loam/instances.h"
#include "loam/DatasetManager.hpp"
#include "loam/matcher/CorrespondenceFinderMatchablesKDtree.hpp"


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

    CorrespondenceVector correspondances;

    CorrespondenceFinderMatchablesKDTreePtr correspondenceFinder =
      CorrespondenceFinderMatchablesKDTreePtr( new CorrespondenceFinderMatchablesKDTree);


    correspondenceFinder->setCorrespondences( &correspondances);
    correspondenceFinder->setEstimate(Isometry3f::Identity() );
    correspondenceFinder->setFixed(&matchables );
    correspondenceFinder->setMoving(&matchables_copy );
    correspondenceFinder->reset();
    correspondenceFinder->compute();



    using FactorType          = SE3Matchable2MatchableErrorFactorNoInfo;
    using FactorBaseType      = Factor_<FactorType::VariableType::BaseVariableType>;

//    const float x = 0;
//    const float y = 0;
//    const float z = 0;
//    const float rx = 0;
//    const float ry = 0.9;
//    const float rz = 0;
//    Vector6f pose;
//    pose << x, y, z, rx, ry, rz;
//    const Isometry3f rotoTransl = srrg2_core::geometry3d::v2t(pose);

    const Isometry3f T = Isometry3f::Identity() * randomRelativeIso();
    for ( auto & m: matchables_copy){
      m.transformInPlace(T);
    }

    const int num_iterations = 10;
    std::vector<FactorBaseType*> factors;
    factors.reserve(correspondances.size());
    for ( auto & c : correspondances){
      FactorType* factor = new FactorType();
      factor->bindFixed(&matchables[c.fixed_idx]);
      factor->bindMoving(&matchables_copy[c.moving_idx]);
      factors.emplace_back(factor);
    }

    const Isometry3f guess = Isometry3f::Identity() * randomRelativeIso();

    SolverDefault_<VariableSE3EulerLeftAD> solver;
    solver.param_max_iterations.pushBack(num_iterations);
    solver.param_termination_criteria.setValue(nullptr);
    solver.clearFactorIterators(); // ia JIC
    solver.addFactorContainer(factors);
    solver.setEstimate(guess);
    solver.compute();
    const auto& stats      = solver.iterationStats();
    const auto& final_chi2 = stats.back().chi_inliers;


    std::cerr << stats << std::endl;
    std::cerr << " final chi : "<< final_chi2 << std::endl;

    const auto& estimated_T = solver.estimate();
    std::cerr << "GT\n" << FG_GREEN(T.matrix()) << std::endl;
    std::cerr << "estimated\n" << FG_YELLOW(estimated_T.matrix()) << std::endl;

   // const auto diff_T      = estimated_T.inverse() * T;
   // const auto diff_vector = geometry3d::t2tnq(diff_T);


    for (size_t i = 0; i < factors.size(); ++i) {
      delete factors[i];
    }

    cloudPtr = dM.readPointerToMessageFromDataset();
  }

  return 0;
}

