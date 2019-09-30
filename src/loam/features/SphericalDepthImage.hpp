#pragma once
#include <srrg_system_utils/system_utils.h>
#include <srrg_messages/instances.h>
#include <random>

#include "opencv2/opencv.hpp"


#include "../include/structs.hpp"
#include "../include/turboColors.hpp"

#include "../MyMath.hpp"
#include "DataPoint.hpp"
#include "IntegralImage.hpp"
#include "Line.hpp"
#include "Plane.hpp"
#include "Clusterer.hpp"

using namespace std;
using namespace srrg2_core;


namespace Loam{


  class SphericalDepthImage {
     protected:
       //this matrix is both the index matrix I and the spherical depth matrix D
       vector<vector< DataPoint >> m_index_image;
       PointNormalColor3fVectorCloud m_cloud;
       sphericalImage_params m_params;
       float m_min_elevation;
       float m_max_elevation;
       Clusterer m_clusterer;
       std::shared_ptr<std::vector<cluster>> m_clustersPtr;
     

     public:


      SphericalDepthImage();

      SphericalDepthImage(
          const PointNormalColor3fVectorCloud & cloud,
          const sphericalImage_params t_params);

      virtual ~SphericalDepthImage();


      void executeOperations();

      void removeFlatSurfaces();
      IntegralImage collectNormals();

      void projectCloud();
      void unprojectCloud();



      void initializeIndexImage();
      void resetIndexImage();

      void markVerticalPoints();

      void removeNonVerticalPoints();
      void removePointsWithoutNormal();


      bool expandNormalBoundariesUp(DataPoint & t_starting_point,int & t_neighboors_count);
      bool expandNormalBoundariesDown(DataPoint & t_starting_point,int & t_neighboors_count);
      bool expandNormalBoundariesLeft(DataPoint & t_starting_point,int & t_neighboors_count);
      bool expandNormalBoundariesRight(DataPoint & t_starting_point,int & t_neighboors_count);
      void discoverNormalsBoundaryIndexes();

      //todo decide wheter to remove the deprecated method and all the integral image logic or to return to the deprecated version
      IntegralImage computePointNormals();
      IntegralImage computePointNormalsDeprecated();


      PointNormalColor3fVectorCloud fetchPoints(const vector<int> & indexes);

      vector< Vector3f>  fetchPointsInBoundaries(
          const int t_rowMin,const int t_rowMax,const int t_colMin,const int t_colMax);
 
      void clusterizeCloud( MatchablePtrVecPtr  t_matchablesPtrVecPtr );

      int countPointsValid();

      vector<int> mapSphericalCoordsInIndexImage(
          const float t_azimuth, const float t_elevation);

      vector<int> mapCartesianCoordsInIndexImage(
          const Vector3f & t_coords);
 
      static Vector2f extimateMinMaxElevation( const PointNormalColor3fVectorCloud & cloud);


      PointNormalColor3fVectorCloud drawClusters3D();

      RGBImage drawIndexImg();
      RGBImage drawNormalsImg();
      RGBImage drawClustersImg();
      vector<RGBImage> drawImgsClusterer();



      RGBImage drawPointNormalBoundaries( const PointNormalColor3f & t_point );


      inline const vector<vector<DataPoint>> & getIndexImage() const { return m_index_image;};
      inline vector<vector<DataPoint>> & getIndexImage(){ return m_index_image;};

      inline void setIndexImage( const vector<vector<DataPoint>> & t_indexImage){ m_index_image = t_indexImage;};

      inline const PointNormalColor3fVectorCloud & getPointCloud() const { return m_cloud;};
      inline PointNormalColor3fVectorCloud & getPointCloud() { return m_cloud;};

      inline void setPointCloud( const PointNormalColor3fVectorCloud & t_cloud){ m_cloud = t_cloud;};

      inline DataPoint getDataPointAt( const int t_row, const int t_col){ return m_index_image[t_row][t_col];};
   };
}


