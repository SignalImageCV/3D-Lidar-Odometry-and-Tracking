#pragma once
#include <srrg_system_utils/system_utils.h>
#include <srrg_messages/instances.h>




namespace Loam{
  using namespace std;
  using namespace srrg2_core;

  typedef struct sphericalDepthPoint_tag{
    int index_in_stl_container;
    Vector3f spherical_coords;
    bool isVertical;
    sphericalDepthPoint_tag():
      index_in_stl_container(-1),
      spherical_coords(Vector3f::Zero()),
      isVertical( false)
    {}
    sphericalDepthPoint_tag( int t_index, Vector3f t_sph_coords):
      index_in_stl_container( t_index),
      spherical_coords( t_sph_coords),
      isVertical( false)
    {}
    sphericalDepthPoint_tag( int t_index,
        Vector3f t_sph_coords, bool t_isVertical):
      index_in_stl_container( t_index),
      spherical_coords( t_sph_coords),
      isVertical( t_isVertical)
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
       float m_epsilon_radius;
       int m_epsilon_times;
       Point3fVectorCloud m_cloud;

     public:
      SphericalDepthImage()= default;

      SphericalDepthImage(
          int num_vertical_rings,
          int num_points_ring,
          float epsilon_radius,
          int epsilon_times,
          const Point3fVectorCloud & cloud);


      ~SphericalDepthImage() = default;

      void buildIndexImage();
      void resetIndexImage();

      void removeFlatSurfaces();
      void markVerticalPoints();
      void removeNonVerticalPoints();

      vector<int> mapSphericalCoordsInIndexImage(
          const float t_azimuth, const float t_elevation);
 
      static Vector2f extimateMinMaxElevation( const Point3fVectorCloud & cloud);

      //they maps a point from cartesian coords in spherical ones
      static  Vector3f directMappingFunc(const Vector3f & t_cart_coords);
      //  and from  spherical coords to cartesian ones
      static  Vector3f inverseMappingFunc(const Vector3f & t_spher_coords);


      inline const vector<vector<list<sphericalDepthPoint>>> & getIndexImage(){ return m_index_image;};

      inline void setIndexImage( const vector<vector<list<sphericalDepthPoint>>> & t_indexImage){ m_index_image = t_indexImage;};

      inline const Point3fVectorCloud & getPointCloud(){ return m_cloud;};

      inline void setPointCloud( const Point3fVectorCloud & t_cloud){ m_cloud = t_cloud;};




   };
}


