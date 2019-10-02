#include "Aligner.hpp"
using namespace std;
using namespace srrg2_core;
using namespace srrg2_solver;


namespace Loam{

  Aligner::Aligner(const sphericalImage_params & t_params, const int t_num_iterations_solver ):
    m_params( t_params),
    m_num_iterations_solver( t_num_iterations_solver),
    m_flag_not_started_yet( true),
    m_measurement_adaptor( CustomMeasurementAdaptorPtr(new CustomMeasurementAdaptor))
  {
  };


  Aligner::~Aligner(){};


  Isometry3f Aligner::compute(
      BaseSensorMessagePtr t_raw_data,
      const  Isometry3f & t_initial_guess 
      ){

    CustomMatchablefVectorData  new_matchables;


    m_measurement_adaptor->setMyParams( m_params);
    m_measurement_adaptor->setDest( &new_matchables);
    m_measurement_adaptor->setMeasurement( t_raw_data);
    m_measurement_adaptor->compute();
    m_measurement_adaptor->reset();



    if (not m_flag_not_started_yet){
      CorrespondenceVector correspondances;

      CorrespondenceFinderMatchablesKDTreePtr correspondenceFinder =
        CorrespondenceFinderMatchablesKDTreePtr( new CorrespondenceFinderMatchablesKDTree);


      correspondenceFinder->setCorrespondences( &correspondances);
      correspondenceFinder->setEstimate(t_initial_guess );
      correspondenceFinder->setFixed(&m_previous_matchables);
      correspondenceFinder->setMoving(&new_matchables);
      correspondenceFinder->reset();
      correspondenceFinder->compute();

      std::cout  << "Number of old matchables: ";
      std::cout  << m_previous_matchables.size() <<"\n";

      std::cout  << "Number of new matchables: ";
      std::cout  << new_matchables.size() <<"\n";

      std::cout  << "Number of correspondences : ";
      std::cout  << correspondances.size() <<"\n";




      std::vector<FactorBaseType*> factors;
      factors.reserve(correspondances.size());

      for ( auto & c : correspondances){
        FactorType* factor = new FactorType();
        factor->bindFixed(&m_previous_matchables[c.fixed_idx]);
        factor->bindMoving(&new_matchables[c.moving_idx]);
        factors.emplace_back(factor);
      }

      SolverDefault_<VariableSE3EulerLeftAD> solver;
      solver.param_max_iterations.pushBack(m_num_iterations_solver);
      solver.param_termination_criteria.setValue(nullptr);
      solver.clearFactorIterators(); // ia JIC
      solver.addFactorContainer(factors);
      solver.setEstimate(t_initial_guess);
      solver.compute();

      //potential memory leak, ask if possible to use smart pointers instead of raw ones
      for (size_t i = 0; i < factors.size(); ++i) {
        delete factors[i];
      }


      auto estimated_T = solver.estimate();

      m_previous_matchables = new_matchables;

      return  estimated_T;

    }
    else{
      m_flag_not_started_yet = false;
      m_previous_matchables = new_matchables;
      return Isometry3f::Identity();
    }
  }

  void Aligner::fillClustersPoints(std::shared_ptr<PointNormalColor3fVectorCloud> clusterPointsPtr){
    m_measurement_adaptor->m_sph_ImagePtr->fillWithClusters3D( clusterPointsPtr);
  }
  
}
 
