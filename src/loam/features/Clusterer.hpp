#include <random>
#include "DataPoint.hpp"
#include <srrg_messages/instances.h>

using namespace std;
using namespace srrg2_core;

namespace Loam{

  typedef struct cluster_tag{
    vector<int> indexes;
    Eigen::Vector3f mu;
    Eigen::Matrix3f sigma;
    cluster_tag()
    {
      mu = Vector3f::Zero();
      sigma = Matrix3f::Zero();
    }
  }cluster;



  class Clusterer{
    private:
      PointNormalColor3fVectorCloud  m_cloud;
      vector<vector< DataPoint>>  m_image;

    public:
      Clusterer( 
         const  PointNormalColor3fVectorCloud & t_cloud,
         vector<vector<DataPoint>> t_flattened_index_image); 

      vector<cluster> findClusters();





  };
}
 


