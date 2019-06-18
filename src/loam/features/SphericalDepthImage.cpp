#include "SphericalDepthImage.hpp"

namespace Loam{
  SphericalDepthImage::SphericalDepthImage(
          int num_vertical_rings,
          int num_points_ring,
          float epsilon_radius,
          int epsilon_times,
          const Point3fVectorCloud & t_cloud):
  m_num_vertical_rings( num_vertical_rings),
  m_num_points_ring( num_points_ring),
  m_epsilon_radius( epsilon_radius),
  m_epsilon_times( epsilon_times),
  m_cloud( t_cloud){
    m_index_image.resize( num_vertical_rings);
    for ( auto & v : m_index_image){
      v.resize( num_points_ring);
    }
    Vector2f min_max_elevation = extimateMinMaxElevation( t_cloud);
    m_min_elevation = min_max_elevation.x();
    m_max_elevation = min_max_elevation.y();
  }


  void SphericalDepthImage::buildIndexImage(){
    if (m_cloud.size()>0){
      for (unsigned int i = 0; i<m_cloud.size(); ++i){
        Vector3f spherical_coords = SphericalDepthImage::directMappingFunc( m_cloud[i].coordinates());
        vector<int> coords = mapSphericalCoordsInIndexImage( spherical_coords.x(), spherical_coords.y()); 
        sphericalDepthPoint sp( i, spherical_coords);
        m_index_image[coords[0]][coords[1]].push_back(sp);
      }
    }
  }

  void SphericalDepthImage::resetIndexImage(){
    for ( auto & columns : m_index_image){
      for ( auto & list: columns){
        list.clear();
      }
    }
  }

  void SphericalDepthImage::removeFlatSurfaces(){
    buildIndexImage();
    markVerticalPoints();
    removeNonVerticalPoints();
    resetIndexImage();
    buildIndexImage();
  }


  void SphericalDepthImage::markVerticalPoints(){
    for (unsigned int col= 0; col<m_index_image[0].size(); ++col){
      for (int row = m_index_image.size() -1; row >= 0; --row){
        for ( auto& entry : m_index_image[row][col]){
          if( not entry.isVertical ){
            int num_points_falling_in_projection = 0;
            Vector3f ref_coords = m_cloud[ entry.index_in_stl_container].coordinates();
            vector< vector< int>> curr_indexes;
            for (int curr_row = row; curr_row >= 0; --curr_row){
              int index_in_list = 0;
              for ( auto& curr_entry: m_index_image[curr_row][col]){
                float x_ref = ref_coords[0];
                float y_ref = ref_coords[1];
                Vector3f curr_coords = m_cloud[ curr_entry.index_in_stl_container].coordinates();
                float x_curr = curr_coords[0];
                float y_curr = curr_coords[1];
                float distance = sqrt( pow( x_ref - x_curr, 2)+ pow( y_ref - y_curr, 2));
                if( distance < m_epsilon_radius){
                  ++num_points_falling_in_projection;
                  vector<int> curr_index;
                  curr_index.reserve(3);
                  curr_index.push_back(curr_row);
                  curr_index.push_back(col);
                  curr_index.push_back(index_in_list);
                  curr_indexes.push_back( curr_index);
                }
                ++index_in_list;
              }
            }
            if( num_points_falling_in_projection >= m_epsilon_times){
              for( auto & indexes: curr_indexes){
                int counter = 0;
                for( auto & p: m_index_image[indexes[0]][indexes[1]]){
                  if( counter == indexes[2]){
                    p.isVertical = true;
                  }
                  ++counter;
                }
              }
            }
          }
        }
      }
    }
  }


  void SphericalDepthImage::removeNonVerticalPoints(){
    vector< int> indexes_vertical_points;
    indexes_vertical_points.reserve( m_cloud.size() );
    for (unsigned int row =0; row <m_index_image.size() ; ++row){
      for (unsigned int col=0; col <m_index_image[0].size(); ++col){
        for ( auto& entry : m_index_image[row][col]){
          if( entry.isVertical){
            indexes_vertical_points.push_back( entry.index_in_stl_container);
          }
        }
      }
    }
    Point3fVectorCloud cleanedCloud;
    cleanedCloud.resize( indexes_vertical_points.size());
    for( unsigned int i =0 ; i< cleanedCloud.size(); ++i){
      cleanedCloud[i].coordinates() = m_cloud[indexes_vertical_points[i]].coordinates();
    }
    m_cloud = cleanedCloud;
  }

  vector<int> SphericalDepthImage::mapSphericalCoordsInIndexImage(
      const float t_azimuth, const float t_elevation ){

    const double interval_elevation = static_cast<double>( (m_max_elevation - m_min_elevation) / m_num_vertical_rings );
    const float elevation_normalized = t_elevation - m_min_elevation;
    int u= static_cast<int>( floor( elevation_normalized/ interval_elevation ));
    if ( u == -1){ u= 0; } 
    if ( u == m_num_vertical_rings){ --u;};

    const double interval_azimuth = static_cast<double>( 2*M_PI / m_num_points_ring);
    const double azimuth_normalized = t_azimuth + M_PI;
    int v= static_cast<int>( floor(azimuth_normalized / interval_azimuth));
    if ( v == -1){ v= 0; } 
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

    const float azimuth = atan2( t_cart_coords.y(), t_cart_coords.x());

    const float elevation = atan2(
        sqrt(pow(t_cart_coords.x(),2)+pow(t_cart_coords.y(),2)),
          t_cart_coords.z());

    return Vector3f(
        azimuth,
        elevation,
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

