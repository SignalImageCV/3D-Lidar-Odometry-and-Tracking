#pragma once
#include <srrg_system_utils/system_utils.h>
#include <srrg_messages/instances.h>

#include "DataPoint.hpp"
#include "IntegralImage.hpp"

using namespace std;
using namespace srrg2_core;


namespace Loam{

  typedef struct sphericalImage_params_tag{
     int num_vertical_rings;
     int num_points_ring;
     int epsilon_times;
     float epsilon_radius;
     float depth_differential_threshold;
     int min_neighboors_for_normal;
     sphericalImage_params_tag(
         const int t_num_vertical_rings,
         const int t_num_points_ring,
         const int t_epsilon_times,
         const float t_epsilon_radius,
         const float t_depth_differential_threshold,
         const int t_min_neighboors_for_normal):
       num_vertical_rings( t_num_vertical_rings),
       num_points_ring( t_num_points_ring),
       epsilon_times( t_epsilon_times),
       epsilon_radius( t_epsilon_radius),
       depth_differential_threshold(
           t_depth_differential_threshold),
       min_neighboors_for_normal(
           t_min_neighboors_for_normal)
    {}
     sphericalImage_params_tag():
       num_vertical_rings( -1),
       num_points_ring( -1),
       epsilon_times( -1),
       epsilon_radius( -1),
       depth_differential_threshold(-1),
       min_neighboors_for_normal(-1)
    {}
  }sphericalImage_params;


  class SphericalDepthImage {
     protected:
       //this matrix is both the index matrix I and the spherical depth matrix D
       vector<vector< list< DataPoint >>> m_index_image;
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

      void removeFlatSurfaces();
      void collectNormals();

      void buildIndexImage();
      void resetIndexImage();

      void markVerticalPoints();
      void removeNonVerticalPoints();

      void discoverBoundaryIndexes();
      void removePointsWithoutNormal();
      void computePointNormals();

      bool expandBoundariesUp( DataPoint & t_starting_point,int & t_neighboors_count);
      bool expandBoundariesDown( DataPoint & t_starting_point,int & t_neighboors_count);
      bool expandBoundariesLeft( DataPoint & t_starting_point,int & t_neighboors_count);
      bool expandBoundariesRight( DataPoint & t_starting_point,int & t_neighboors_count);

      vector<int> mapSphericalCoordsInIndexImage(
          const float t_azimuth, const float t_elevation);

      vector<int> mapCartesianCoordsInIndexImage(
          const Vector3f & t_coords);
 
      static Vector2f extimateMinMaxElevation( const PointNormalColor3fVectorCloud & cloud);

      static  Vector3f directMappingFunc(const Vector3f & t_cart_coords);

      static  Vector3f inverseMappingFunc(const Vector3f & t_spher_coords);



      inline const vector<vector<list<DataPoint>>> & getIndexImage(){ return m_index_image;};

      inline void setIndexImage( const vector<vector<list<DataPoint>>> & t_indexImage){ m_index_image = t_indexImage;};

      inline const PointNormalColor3fVectorCloud & getPointCloud(){ return m_cloud;};

      inline void setPointCloud( const PointNormalColor3fVectorCloud & t_cloud){ m_cloud = t_cloud;};

      inline list<DataPoint> getListDataPointsAt( const int t_row, const int t_col){ return m_index_image[t_row][t_col];};
   };
}


