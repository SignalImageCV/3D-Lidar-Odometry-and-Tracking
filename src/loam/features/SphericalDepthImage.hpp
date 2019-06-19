#pragma once
#include <srrg_system_utils/system_utils.h>
#include <srrg_messages/instances.h>

#include "DataPoint.hpp"

using namespace std;
using namespace srrg2_core;


namespace Loam{
  

  class SphericalDepthImage {
     protected:
       //this matrix is both the index matrix I and the spherical depth matrix D
       vector<vector< list< DataPoint >>> m_index_image;
       int m_num_vertical_rings;
       int m_num_points_ring;
       float m_min_elevation;
       float m_max_elevation;
       float m_epsilon_radius;
       int m_epsilon_times;
       PointNormalColor3fVectorCloud m_cloud;

     public:
      SphericalDepthImage()= default;

      SphericalDepthImage(
          int num_vertical_rings,
          int num_points_ring,
          float epsilon_radius,
          int epsilon_times,
          const PointNormalColor3fVectorCloud & cloud);

      ~SphericalDepthImage() = default;

      void buildIndexImage();
      void resetIndexImage();

      void removeFlatSurfaces();
      void markVerticalPoints();
      void removeNonVerticalPoints();

      vector<int> mapSphericalCoordsInIndexImage(
          const float t_azimuth, const float t_elevation);
 
      static Vector2f extimateMinMaxElevation( const PointNormalColor3fVectorCloud & cloud);

      //they maps a point from cartesian coords in spherical ones
      static  Vector3f directMappingFunc(const Vector3f & t_cart_coords);
      //  and from  spherical coords to cartesian ones
      static  Vector3f inverseMappingFunc(const Vector3f & t_spher_coords);


      inline const vector<vector<list<DataPoint>>> & getIndexImage(){ return m_index_image;};

      inline void setIndexImage( const vector<vector<list<DataPoint>>> & t_indexImage){ m_index_image = t_indexImage;};

      inline const PointNormalColor3fVectorCloud & getPointCloud(){ return m_cloud;};

      inline void setPointCloud( const PointNormalColor3fVectorCloud & t_cloud){ m_cloud = t_cloud;};

   };
}


