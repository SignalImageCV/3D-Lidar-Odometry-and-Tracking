#include "Clusterer.hpp"

namespace Loam{

  Clusterer::Clusterer( 
         const  PointNormalColor3fVectorCloud & t_cloud,
         vector<vector<DataPoint>> t_flattened_index_image):
    m_cloud(t_cloud), m_image( t_flattened_index_image)
  {}


  vector<cluster> Clusterer::findClusters(){
    vector<cluster> clusters;


    return clusters;
  }














}


