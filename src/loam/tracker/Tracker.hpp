#pragma once
#include "loam/aligner/Aligner.hpp"
#include "Map.hpp"
#include "loam/DatasetManager.hpp"


namespace Loam{

  using namespace srrg2_core;

  class Tracker{
    private:
      Aligner m_aligner;
      DatasetManager m_datasetManager;
      Map m_map;
      Isometry3f m_absolute_transformation;
      Isometry3f m_relative_transformation;


    public:
      std::shared_ptr<PointNormalColor3fVectorCloud> m_current_clusterPointsPtr;

      Tracker(  const std::string & t_filePath, const sphericalImage_params & t_params, const int t_num_iterations_solver = 10 );

      virtual ~Tracker();

      void executeCycle();


      inline const Isometry3f & getRelativeT() const { return m_relative_transformation;};
      inline Isometry3f & getRelativeT(){ return m_relative_transformation;};

      inline const Isometry3f & getAbsoluteT() const { return m_absolute_transformation;};
      inline Isometry3f & getAbsoluteT(){ return m_absolute_transformation;};
  };

}

