#pragma once
#include <srrg_system_utils/system_utils.h>
#include <srrg_messages/instances.h>
#include <random>

#include "opencv2/opencv.hpp"


#include "../include/structs.hpp"
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
       //vector<vector< list< DataPoint >>> m_index_image;
       vector<vector< DataPoint >> m_index_image;
       PointNormalColor3fVectorCloud m_cloud;
       sphericalImage_params m_params;
       float m_min_elevation;
       float m_max_elevation;
     

     public:
      SphericalDepthImage()= default;

      SphericalDepthImage(
          const PointNormalColor3fVectorCloud & cloud,
          const sphericalImage_params t_params);

      ~SphericalDepthImage() = default;

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

      IntegralImage computePointNormals();


  



      vector<Matchable> clusterizeCloud();
        


      /////////////////////////////////// DEPRECATED begin

      vector<Matchable> clusterizeCloudDeprecated(IntegralImage & t_integ_img);

      vector< vector< int>> findGoodClusterSeeds();

      bool expandClusterBoundariesUp( DataPoint & t_seed_point,int & t_included_points_count);
      bool expandClusterBoundariesDown( DataPoint & t_seed_point,int & t_included_points_count);
      bool expandClusterBoundariesLeft( DataPoint & t_seed_point,int & t_included_points_count);
      bool expandClusterBoundariesRight( DataPoint & t_seed_point,int & t_included_points_count);
      bool discoverClusterBoundaryIndexes(
          const int t_seed_row, const int t_seed_col, const int t_seed_list_position);


      void markClusteredPoints(DataPoint & t_seed_point);

      ////////////////////////////// end
      vector<int> fetchGoodSeedIndexes();

      int countPointsIndexImage();
      int countPointsNotClustered();

      PointNormalColor3fVectorCloud fetchPointsInBoundaries(
          const int t_rowMin,const int t_rowMax,const int t_colMin,const int t_colMax);
 


      vector<int> mapSphericalCoordsInIndexImage(
          const float t_azimuth, const float t_elevation);

      vector<int> mapCartesianCoordsInIndexImage(
          const Vector3f & t_coords);
 
      static Vector2f extimateMinMaxElevation( const PointNormalColor3fVectorCloud & cloud);

      static  Vector3f directMappingFunc(const Vector3f & t_cart_coords);

      static  Vector3f inverseMappingFunc(const Vector3f & t_spher_coords);


      RGBImage drawIndexImg();
      RGBImage drawNormalsImg();
      RGBImage drawClustersImg();



      inline const vector<vector<DataPoint>> & getIndexImage(){ return m_index_image;};

      inline void setIndexImage( const vector<vector<DataPoint>> & t_indexImage){ m_index_image = t_indexImage;};

      inline const PointNormalColor3fVectorCloud & getPointCloud(){ return m_cloud;};

      inline void setPointCloud( const PointNormalColor3fVectorCloud & t_cloud){ m_cloud = t_cloud;};

      inline DataPoint getDataPointAt( const int t_row, const int t_col){ return m_index_image[t_row][t_col];};
   };
}


