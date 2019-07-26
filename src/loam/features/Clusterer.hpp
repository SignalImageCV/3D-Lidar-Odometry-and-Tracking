#pragma once
#include <random>
#include <limits>
#include <stack>
#include <srrg_messages/instances.h>
#include "DataPoint.hpp"
#include "SphericalDepthImage.hpp"

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

  typedef struct matCoords_tag{
    int row;
    int col;
    matCoords_tag():row(-1),col(-1)
    {}
    matCoords_tag(
        const int t_row,
        const int t_col):
      row(t_row),col(t_col)
    {}
  }matCoords;

  typedef struct pathCell_tag{
    bool hasBeenChoosen;
    float depth;
    matCoords matCoords;
    pathCell_tag():
      hasBeenChoosen(false),
      depth( numeric_limits<float>::max())
    {}
    pathCell_tag(const int row, const int col,  const float t_depth ):
      hasBeenChoosen(false),
      depth( t_depth),
      matCoords(row, col)
    {}
  }pathCell;




  class Clusterer{
    private:
      PointNormalColor3fVectorCloud  m_cloud;
      vector<vector< DataPoint>>  m_image;
      vector<vector< pathCell>>  m_pathMatrix;
      sphericalImage_params m_params;

    public:
      Clusterer( 
         const  PointNormalColor3fVectorCloud & t_cloud,
         vector<vector<list<DataPoint>>> t_index_image,
         const sphericalImage_params t_params);
         


       void flattenIndexImage_populatePathMatrix(
         const  PointNormalColor3fVectorCloud & t_cloud,
         vector<vector<list<DataPoint>>> t_index_image);



      matCoords findSeed();

      vector<pathCell> findNeighboors(pathCell & t_pathCell);
      cluster computeCluster(const matCoords & t_seed_coords);
      vector<cluster> findClusters();



  };
}
 


