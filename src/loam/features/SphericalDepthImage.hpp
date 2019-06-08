#pragma once
#include <srrg_system_utils/system_utils.h>
#include <srrg_messages/instances.h>




namespace Loam{
  using namespace std;
  using namespace srrg2_core;


   class SphericalDepthImage {
     protected:
       vector<  vector< list<int> >> m_index_image;
       int m_num_vertical_rings;
       int m_num_points_ring;
       float m_min_elevation;
       float m_max_elevation;

     public:
      SphericalDepthImage() = default;

      SphericalDepthImage( int num_vertical_rings, int num_points_ring, const Point3fVectorCloud & cloud);

      ~SphericalDepthImage() = default;


      vector<int> mapSphericalCoordsInIndexImage(
          const float t_azimuth, const float t_elevation);
 
      static Vector2f extimateMinMaxElevation( const Point3fVectorCloud & cloud);

      //they maps a point from cartesian coords in spherical ones
      static  Vector3f directMappingFunc(const Vector3f & t_cart_coords);
      //  and from  spherical coords to cartesian ones
      static  Vector3f inverseMappingFunc(const Vector3f & t_spher_coords);





   };
}


