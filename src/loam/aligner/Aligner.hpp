#include <srrg_messages/instances.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_solver/solver_core/instances.h>
#include <srrg_solver/SE3/instances.h>

#include "loam/instances.h"
#include "loam/CustomMeasurementAdaptor.hpp"
//#include "loam/matcher/CorrespondenceFinderMatchablesKDtree.hpp"
#include "loam/matcher/CorrespondenceFinderMatchablesBruteForce.hpp"


using namespace srrg2_solver;
namespace Loam{
    class Aligner{
    protected:
      sphericalImage_params m_params;
      int m_num_iterations_solver;
      bool m_flag_not_started_yet;
      CustomMeasurementAdaptorPtr  m_measurement_adaptor;
      CustomMatchablefVectorData m_previous_matchables;
    public:
      using FactorType          = SE3Matchable2MatchableErrorFactorNoInfo;
      using FactorBaseType      = Factor_<FactorType::VariableType::BaseVariableType>;

      Aligner(const sphericalImage_params & t_params, const int t_num_iterations_solver = 10 );

      virtual ~Aligner();

      Isometry3f compute(
          BaseSensorMessagePtr t_raw_data,
          const  Isometry3f & t_initial_guess =Isometry3f::Identity()
          );

      void fillClustersPoints( std::shared_ptr<PointNormalColor3fVectorCloud> clusterPointsPtr);
  };
}


