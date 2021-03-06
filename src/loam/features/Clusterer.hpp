#pragma once
#include <random>
#include <limits>
#include <stack>
#include <srrg_messages/instances.h>

#include "../include/structs.hpp"
#include "DataPoint.hpp"

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

  typedef struct matrixCoords_tag{
    int row;
    int col;
    matrixCoords_tag():row(-1),col(-1)
    {}
    matrixCoords_tag(
        const int t_row,
        const int t_col):
      row(t_row),col(t_col)
    {}
  }matrixCoords;

  typedef struct pathCell_tag{
    bool hasBeenChosen;
    float depth;
    Eigen::Vector3f normal;
    matrixCoords matCoords;
    pathCell_tag():
      hasBeenChosen(false),
      depth( numeric_limits<float>::max()),
      normal( Eigen::Vector3f::Zero())
    {}
    pathCell_tag(const int row, const int col,
        const float t_depth, const Eigen::Vector3f t_normal ):
      hasBeenChosen(false),
      depth( t_depth),
      normal( t_normal),
      matCoords(row, col)
    {}
  }pathCell;




  class Clusterer{
    private:
      PointNormalColor3fVectorCloud  m_cloud;
      vector<vector< DataPoint>>  m_index_image;
      sphericalImage_params m_params;
      vector<vector< Eigen::Vector3f>>  m_blurredNormalsMatrix;
      vector<vector< pathCell>>  m_pathMatrix;
      int m_blur_extension;
      int m_neigh_ext_vertical;
      int m_neigh_ext_horizontal;

    public:

      Clusterer()= default;

      Clusterer( 
         const PointNormalColor3fVectorCloud & t_cloud,
         const vector<vector<DataPoint>> & t_index_image,
         const sphericalImage_params t_params);

      ~Clusterer() = default;
         
      static vector<vector<pathCell>> populatePathMatrix(
         const PointNormalColor3fVectorCloud & t_cloud,
         const vector<vector<DataPoint>> & t_index_image,
         const sphericalImage_params t_params);

      void blurNormals();


      matrixCoords findSeed();

      vector<pathCell> findNeighboors(pathCell & t_pathCell);
      cluster computeCluster(const matrixCoords & t_seed_coords);
      void findClusters( std::shared_ptr<std::vector<cluster>> t_clusters );

      RGBImage drawPathImg();

      RGBImage drawBlurredNormalsImg();
  };
}
 


