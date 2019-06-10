#pragma once
#include <srrg_system_utils/system_utils.h>
#include <srrg_messages/instances.h>




namespace Loam{
  using namespace std;
  using namespace srrg2_core;

  typedef struct sphericalDepthPoint_tag{
    int index_in_stl_container;
    Vector3f spherical_coords;
    sphericalDepthPoint_tag( int t_index, Vector3f t_sph_coords):
      index_in_stl_container( t_index),
      spherical_coords( t_sph_coords)
    {}
  }sphericalDepthPoint;


  class SphericalDepthImage {
     protected:
       //this matrix is both the index matrix I and the spherical depth matrix D
       vector<vector< list< sphericalDepthPoint >>> m_index_image;
       int m_num_vertical_rings;
       int m_num_points_ring;
       float m_min_elevation;
       float m_max_elevation;

     public:
      SphericalDepthImage() = default;

      SphericalDepthImage( int num_vertical_rings, int num_points_ring, const Point3fVectorCloud & cloud);

      ~SphericalDepthImage() = default;

      void buildIndexImage(const Point3fVectorCloud & t_cloud);

      vector<int> mapSphericalCoordsInIndexImage(
          const float t_azimuth, const float t_elevation);
 
      static Vector2f extimateMinMaxElevation( const Point3fVectorCloud & cloud);

      //they maps a point from cartesian coords in spherical ones
      static  Vector3f directMappingFunc(const Vector3f & t_cart_coords);
      //  and from  spherical coords to cartesian ones
      static  Vector3f inverseMappingFunc(const Vector3f & t_spher_coords);


      inline const vector<vector<list<sphericalDepthPoint>>> & getIndexImage(){ return m_index_image;};






   };
}


