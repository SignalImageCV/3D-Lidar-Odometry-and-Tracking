#include "SphericalDepthImage.hpp"

namespace Loam{
    
  SphericalDepthImage::SphericalDepthImage( int num_vertical_rings ,
      int num_points_ring , const Point3fVectorCloud & cloud):
  m_num_vertical_rings( num_vertical_rings), m_num_points_ring( num_points_ring){
    m_index_image.resize( num_vertical_rings);
    for ( auto & v : m_index_image){
      v.resize( num_points_ring);
    }
    Vector2f min_max_elevation = extimateMinMaxElevation( cloud);
    m_min_elevation = min_max_elevation.x();
    m_max_elevation = min_max_elevation.y();
  }


  void SphericalDepthImage::buildIndexImage(const Point3fVectorCloud & t_cloud){
    if (t_cloud.size()>0){

      for ( int i = 0; i<t_cloud.size(); ++i){
        Vector3f spherical_coords = SphericalDepthImage::directMappingFunc( t_cloud[i].coordinates());
        vector<int> coords = mapSphericalCoordsInIndexImage( spherical_coords.x(), spherical_coords.y()); 
        m_index_image[coords[0]][coords[1]].push_back(i);
      }
    }
  }

  vector<int> SphericalDepthImage::mapSphericalCoordsInIndexImage(
      const float t_azimuth, const float t_elevation ){

    const double interval_elevation = static_cast<double>( (m_max_elevation - m_min_elevation) / m_num_vertical_rings );
    const float elevation_normalized = t_elevation - m_min_elevation;
    int u= static_cast<int>( floor( elevation_normalized/ interval_elevation ));
    if ( u == m_num_vertical_rings){ --u;};

    const double interval_azimuth = static_cast<double>( 2*M_PI / m_num_points_ring);
    const double azimuth_normalized = t_azimuth + M_PI;
    int v= static_cast<int>( floor(azimuth_normalized / interval_azimuth));
    if ( v == m_num_points_ring){ --v;};

    vector<int> result;
    result.push_back(u);
    result.push_back(v);
    return result;
  }

  Vector2f SphericalDepthImage::extimateMinMaxElevation( const Point3fVectorCloud & cloud){
    float min_elevation = 0;
    float max_elevation = 0;
    if (cloud.size()>0){
      Vector3f initialSphericalCoords = SphericalDepthImage::directMappingFunc( cloud[0].coordinates());
      min_elevation = initialSphericalCoords.y();
      max_elevation = initialSphericalCoords.y();
      Vector3f curr_sphericalCoords;
      for ( auto & p: cloud){
        curr_sphericalCoords = SphericalDepthImage::directMappingFunc( p.coordinates());
        if( curr_sphericalCoords.y()< min_elevation){
          min_elevation = curr_sphericalCoords.y();
        }
        if( curr_sphericalCoords.y()> max_elevation){
          max_elevation = curr_sphericalCoords.y();
        }
      }
    }
    return  Vector2f( min_elevation, max_elevation);
  }

  Vector3f SphericalDepthImage::directMappingFunc(const Vector3f & t_cart_coords){
    return Vector3f(
        atan2( t_cart_coords.y(), t_cart_coords.x()),
        atan2( sqrt(pow(t_cart_coords.x(),2)+pow(t_cart_coords.y(),2)),
          t_cart_coords.z()),
        sqrt( pow(t_cart_coords.x(),2)+
          pow(t_cart_coords.y(),2)+
          pow( t_cart_coords.z(),2))
        );
  };

  Vector3f SphericalDepthImage::inverseMappingFunc(const Vector3f & t_spher_coords){
    float z = t_spher_coords.z() * cos( t_spher_coords.y());
    float proj = t_spher_coords.z() * sin( t_spher_coords.y());
    float x = proj * cos( t_spher_coords.x());
    float y = proj * sin( t_spher_coords.x());
    return Vector3f( x,y,z) ;
  };


}

