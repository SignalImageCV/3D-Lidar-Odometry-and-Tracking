#include "Tracker.hpp"

namespace Loam{

  Tracker::Tracker( const std::string & t_filePath,const sphericalImage_params & t_params, const int t_num_iterations_solver ):
    m_aligner( Aligner( t_params, t_num_iterations_solver )),
    m_datasetManager( t_filePath),
    m_map(),
    m_absolute_transformation( Isometry3f::Identity()),
    m_relative_transformation( Isometry3f::Identity())
    {};

  Tracker::~Tracker(){};

  void Tracker::executeCycle(){

    BaseSensorMessagePtr cloudPtr = m_datasetManager.readPointerToMessageFromDataset();

    if( cloudPtr ) {
      m_relative_transformation =   m_aligner.compute(cloudPtr, m_relative_transformation);

      m_absolute_transformation = m_absolute_transformation * m_relative_transformation;

      m_current_clusterPointsPtr = std::make_shared<PointNormalColor3fVectorCloud>();

      m_aligner.fillClustersPoints(m_current_clusterPointsPtr);

      m_map.insertPoints( *m_current_clusterPointsPtr, m_absolute_transformation);
    }
  };

}
