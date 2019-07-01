#include "SphericalDepthImage.hpp"

namespace Loam{
  SphericalDepthImage::SphericalDepthImage(
          const PointNormalColor3fVectorCloud  & t_cloud,
          const sphericalImage_params t_params
          ):
  m_cloud( t_cloud),
  m_params( t_params)
  {
    m_index_image.resize( t_params.num_vertical_rings);
    for ( auto & v : m_index_image){
      v.resize( t_params.num_points_ring);
    }
    Vector2f min_max_elevation = extimateMinMaxElevation( t_cloud);
    m_min_elevation = min_max_elevation.x();
    m_max_elevation = min_max_elevation.y();
  }

  void SphericalDepthImage::executeOperations(){
    removeFlatSurfaces();
    collectNormals();
  }

  void SphericalDepthImage::removeFlatSurfaces(){
    initializeIndexImage();
    markVerticalPoints();
    removeNonVerticalPoints();
  }

  void SphericalDepthImage::collectNormals(){
    discoverBoundaryIndexes();
    removePointsWithoutNormal();
    computePointNormals();
    
 }


  void SphericalDepthImage::initializeIndexImage(){
    if (m_cloud.size()>0){
      for (unsigned int i = 0; i<m_cloud.size(); ++i){
        vector<int> coords = mapCartesianCoordsInIndexImage( m_cloud[i].coordinates());
        DataPoint dp( i);
        m_index_image[coords[0]][coords[1]].push_back(dp);
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

  void SphericalDepthImage::markVerticalPoints(){
    for (unsigned int col= 0; col<m_index_image[0].size(); ++col){
      for (int row = m_index_image.size() -1; row >= 0; --row){
        for ( auto& entry : m_index_image[row][col]){
          if( not entry.getIsVertical()){
            int num_points_falling_in_projection = 0;
            Vector3f ref_coords = m_cloud[ entry.getIndexContainer()].coordinates();
            vector< vector< int>> curr_indexes;
            for (int curr_row = row; curr_row >= 0; --curr_row){
              int index_in_list = 0;
              for ( auto& curr_entry: m_index_image[curr_row][col]){
                float x_ref = ref_coords[0];
                float y_ref = ref_coords[1];
                Vector3f curr_coords = m_cloud[ curr_entry.getIndexContainer()].coordinates();
                float x_curr = curr_coords[0];
                float y_curr = curr_coords[1];
                float distance = sqrt( pow( x_ref - x_curr, 2)+ pow( y_ref - y_curr, 2));
                if( distance < m_params.epsilon_radius){
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
            if( num_points_falling_in_projection >= m_params.epsilon_times){
              for( auto & indexes: curr_indexes){
                int counter = 0;
                for( auto & p: m_index_image[indexes[0]][indexes[1]]){
                  if( counter == indexes[2]){
                    p.setIsVertical( true);
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

    vector<vector< list< DataPoint >>> new_index_img;
    new_index_img.resize( m_params.num_vertical_rings);
    for ( auto & v : new_index_img){
      v.resize( m_params.num_points_ring);
    }
   
    int index_container = 0;
    PointNormalColor3fVectorCloud cleanedCloud;
    cleanedCloud.reserve( m_cloud.size());

    for (unsigned int row =0; row <m_index_image.size() ; ++row){
      for (unsigned int col=0; col <m_index_image[0].size(); ++col){
        for ( auto& entry : m_index_image[row][col]){
          if( entry.getIsVertical()){
            DataPoint copyOfEntry = DataPoint(entry);
            copyOfEntry.setIndexContainer( index_container);
            cleanedCloud.push_back( m_cloud[entry.getIndexContainer()]);
            new_index_img[row][col].push_back( copyOfEntry);
            ++index_container;
          }
        }
      }
    }
    m_cloud = cleanedCloud;
    m_index_image = new_index_img;
  }



  void SphericalDepthImage::removePointsWithoutNormal(){

    vector<vector< list< DataPoint >>> new_index_img;
    new_index_img.resize( m_params.num_vertical_rings);
    for ( auto & v : new_index_img){
      v.resize( m_params.num_points_ring);
    }

    int index_container = 0;
    PointNormalColor3fVectorCloud cleanedCloud;
    cleanedCloud.reserve( m_cloud.size());
   
    for (unsigned int row =0; row <m_index_image.size() ; ++row){
      for (unsigned int col=0; col <m_index_image[0].size(); ++col){
        for ( auto& entry : m_index_image[row][col]){
          if( entry.getHasNormal()){
            DataPoint copyOfEntry = DataPoint(entry);
            copyOfEntry.setIndexContainer( index_container);
            cleanedCloud.push_back( m_cloud[entry.getIndexContainer()]);
            new_index_img[row][col].push_back( copyOfEntry);
            ++index_container;
          }
        }
      }
    }
    m_cloud = cleanedCloud;
    m_index_image = new_index_img;
  }

  bool SphericalDepthImage::expandBoundariesUp( DataPoint & t_starting_point, int & t_neighboors_count){

    vector<int> curr_boundaries = t_starting_point.getBoundaries();

    const int old_row_min = curr_boundaries[0];
    const int old_row_max = curr_boundaries[1];
    const int old_col_min = curr_boundaries[2];
    const int old_col_max = curr_boundaries[3];
    const int new_row_min = old_row_min - 1;
    if( new_row_min < 0){
      return false;
    }

    const Eigen::Vector3f starting_cartesian_coords =
      m_cloud[t_starting_point.getIndexContainer()].coordinates();
    const Eigen::Vector3f starting_spherical_coords =
      SphericalDepthImage::directMappingFunc( starting_cartesian_coords);
    bool hasExpanded= true;

    int current_added_neighboors = 0;
    for( unsigned int c = old_col_min; c < old_col_max; ++c){
      for( auto & entry: m_index_image[new_row_min][c]){
        const Eigen::Vector3f other_cartesian_coords =
          m_cloud[ entry.getIndexContainer()].coordinates();
        const Eigen::Vector3f other_spherical_coords =
          SphericalDepthImage::directMappingFunc( other_cartesian_coords);

        if ( abs( starting_spherical_coords.z() - other_spherical_coords.z()) >
            m_params.depth_differential_threshold){
          hasExpanded= false;
        }
        else{
          ++current_added_neighboors;
        }
      }
    }
    if (hasExpanded){
      vector<int> new_boundaries = { new_row_min, old_row_max, old_col_min, old_col_max };
      t_starting_point.setBoundaries( new_boundaries);
      t_neighboors_count += current_added_neighboors;
    }
    return hasExpanded;
  }




  bool SphericalDepthImage::expandBoundariesDown( DataPoint & t_starting_point,int & t_neighboors_count){

    vector<int> curr_boundaries = t_starting_point.getBoundaries();

    const int old_row_min = curr_boundaries[0];
    const int old_row_max = curr_boundaries[1];
    const int old_col_min = curr_boundaries[2];
    const int old_col_max = curr_boundaries[3];
    const int new_row_max = old_row_max + 1;
    if( new_row_max >= m_index_image.size()){
      return false;
    }


    const Eigen::Vector3f starting_cartesian_coords =
      m_cloud[t_starting_point.getIndexContainer()].coordinates();
    const Eigen::Vector3f starting_spherical_coords =
      SphericalDepthImage::directMappingFunc( starting_cartesian_coords);
    bool hasExpanded= true;

    int current_added_neighboors = 0;
    for( unsigned int c = old_col_min; c < old_col_max; ++c){
      for( auto & entry: m_index_image[new_row_max][c]){
        const Eigen::Vector3f other_cartesian_coords =
          m_cloud[ entry.getIndexContainer()].coordinates();
        const Eigen::Vector3f other_spherical_coords =
          SphericalDepthImage::directMappingFunc( other_cartesian_coords);

        if ( abs( starting_spherical_coords.z() - other_spherical_coords.z()) >
            m_params.depth_differential_threshold){
          hasExpanded= false;
        }
        else{
          ++current_added_neighboors;
        }
      }
    }
    if (hasExpanded){
      vector<int> new_boundaries = { old_row_min, new_row_max, old_col_min, old_col_max };
      t_starting_point.setBoundaries( new_boundaries);
      t_neighboors_count += current_added_neighboors;
    }
    return hasExpanded;
  }


  bool SphericalDepthImage::expandBoundariesRight( DataPoint & t_starting_point,int & t_neighboors_count){

    vector<int> curr_boundaries = t_starting_point.getBoundaries();

    const int old_row_min = curr_boundaries[0];
    const int old_row_max = curr_boundaries[1];
    const int old_col_min = curr_boundaries[2];
    const int old_col_max = curr_boundaries[3];
    int new_col_min = old_col_min - 1;
    if( new_col_min <0 ){
      new_col_min = m_index_image[0].size()-1;
    }
    if( new_col_min == old_col_max){
      return false;
    }


    const Eigen::Vector3f starting_cartesian_coords =
      m_cloud[t_starting_point.getIndexContainer()].coordinates();
    const Eigen::Vector3f starting_spherical_coords =
      SphericalDepthImage::directMappingFunc( starting_cartesian_coords);
    bool hasExpanded= true;

    int current_added_neighboors = 0;
    for( unsigned int r = old_row_min; r < old_row_max; ++r){
      for( auto & entry: m_index_image[r][new_col_min]){
        const Eigen::Vector3f other_cartesian_coords =
          m_cloud[ entry.getIndexContainer()].coordinates();
        const Eigen::Vector3f other_spherical_coords =
          SphericalDepthImage::directMappingFunc( other_cartesian_coords);

        if ( abs( starting_spherical_coords.z() - other_spherical_coords.z()) >
           m_params.depth_differential_threshold ){
          hasExpanded= false;
        }
        else{
          ++current_added_neighboors;
        }
      }
    }
    if (hasExpanded){
      vector<int> new_boundaries = { old_row_min, old_row_max, new_col_min, old_col_max };
      t_starting_point.setBoundaries( new_boundaries);
      t_neighboors_count += current_added_neighboors;
    }
    return hasExpanded;
  }


  bool SphericalDepthImage::expandBoundariesLeft( DataPoint & t_starting_point,int & t_neighboors_count){

    vector<int> curr_boundaries = t_starting_point.getBoundaries();

    const int old_row_min = curr_boundaries[0];
    const int old_row_max = curr_boundaries[1];
    const int old_col_min = curr_boundaries[2];
    const int old_col_max = curr_boundaries[3];
    int new_col_max = old_col_max + 1;

    if( new_col_max  >= m_index_image[0].size()){
      new_col_max = 0;
    }
    if( new_col_max == old_col_min){
      return false;
    }

    const Eigen::Vector3f starting_cartesian_coords =
      m_cloud[t_starting_point.getIndexContainer()].coordinates();
    const Eigen::Vector3f starting_spherical_coords =
      SphericalDepthImage::directMappingFunc( starting_cartesian_coords);
    bool hasExpanded= true;
    int current_added_neighboors = 0;

    for( unsigned int r = old_row_min; r < old_row_max; ++r){
      for( auto & entry: m_index_image[r][new_col_max]){
        const Eigen::Vector3f other_cartesian_coords =
          m_cloud[ entry.getIndexContainer()].coordinates();
        const Eigen::Vector3f other_spherical_coords =
          SphericalDepthImage::directMappingFunc( other_cartesian_coords);

        if ( abs( starting_spherical_coords.z() - other_spherical_coords.z()) >
            m_params.depth_differential_threshold ){
          hasExpanded= false;
        }
        else{
          ++current_added_neighboors;
        }
      }
    }
    if (hasExpanded){
      vector<int> new_boundaries = { old_row_min, old_row_max, old_col_min, new_col_max };
      t_starting_point.setBoundaries( new_boundaries);
      t_neighboors_count += current_added_neighboors;
    }
    return hasExpanded;
  }

  void SphericalDepthImage::discoverBoundaryIndexes(){

    for (int row =0; row <m_index_image.size() ; ++row){
      for (int col=0; col <m_index_image[0].size(); ++col){
        for ( auto& entry : m_index_image[row][col]){
          vector<int> curr_boundaries = { row, row, col, col};
          entry.setBoundaries( curr_boundaries);
          int neighboors_count = 0;
          bool topFree = true;
          bool downFree = true;
          bool leftFree = true;
          bool rightFree = true;
          int direction_counter = 0;
          int iteration_counter= 0;
          const int iteration_limit = m_index_image.size()+ m_index_image[0].size() ;
          while (
              neighboors_count <  m_params.min_neighboors_for_normal and
              (topFree or downFree or leftFree or rightFree) and
              iteration_counter < iteration_limit
              ){
            switch( direction_counter % 4){
              case(0):
               topFree =  expandBoundariesUp( entry, neighboors_count);
               break;
              case(1):
               downFree =  expandBoundariesDown( entry, neighboors_count);
               break;
              case(2):
               leftFree =  expandBoundariesLeft( entry, neighboors_count);
               break;
              case(3):
               rightFree =  expandBoundariesRight( entry, neighboors_count);
               break;
            }
            ++direction_counter;
            ++iteration_counter;
          }
          entry.setHasNormal(neighboors_count >=  m_params.min_neighboors_for_normal);
        }
      }
    }
  }

  void  SphericalDepthImage::computePointNormals(){
    IntegralImage integ_img = IntegralImage( m_cloud, m_index_image);
    integ_img.buildIntegMatrix();
    
    for (int row =0; row <m_index_image.size() ; ++row){
      for (int col=0; col <m_index_image[0].size(); ++col){
        for ( auto& entry : m_index_image[row][col]){
          int min_row = entry.getBoundaries()[0];
          int max_row = entry.getBoundaries()[1];
          int min_col = entry.getBoundaries()[2];
          int max_col = entry.getBoundaries()[3];

          IntegralCell cell = integ_img.getCellInsideBoundaries( min_row, max_row, min_col, max_col);
          Eigen::Vector3f  p_sum = cell.getPsum();
          Eigen::Matrix3f p_quad = cell.getPquad();
          int p_num = cell.getPnum();

          Eigen::Vector3f mu = p_sum/p_num;
          Eigen::Matrix3f S = p_quad/p_num;
          Eigen::Matrix3f sigma = S - mu * mu.transpose();
          Eigen::JacobiSVD<Eigen::Matrix3f> svd(sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);

          Eigen::Matrix3f R = svd.matrixU();
          Eigen::Vector3f smallestEigenVec= R.row(2);

          Eigen::Vector3f p_coords =  m_cloud[ entry.getIndexContainer()].coordinates();
          if ( (p_coords + smallestEigenVec).norm() >  p_coords.norm() ){
            m_cloud[ entry.getIndexContainer()].normal() =  - smallestEigenVec;
          }
          else{
            m_cloud[ entry.getIndexContainer()].normal() = smallestEigenVec;
          }
        }
      }
    }
  }

  vector<Matchable>  SphericalDepthImage::clusterizeCloud(){
    vector<Matchable> matchables;

    // choose if : make interalimage a class member or returning it from
    // the compute normal method and require it as a param in clusterizecloud

    // expand boundaries with an alternative decision method and confronting
    // the border with the new points not only the center as before
    // this implies 4 different expansion funcs ( code duplication) 
    // and another findBoundaries func ( code duplication)
    

    // use a strategy to keep trak of the points already considered
    // they will be choosen randomly, maybe keep a vector of indexes
    // of the already chosen to use afterward
    //
    //
    //
    return matchables;
  }

  vector<int> SphericalDepthImage::mapSphericalCoordsInIndexImage(
      const float t_azimuth, const float t_elevation ){

    const double interval_elevation = static_cast<double>(
        (m_max_elevation - m_min_elevation) / m_params.num_vertical_rings);
    const float elevation_normalized = t_elevation - m_min_elevation;
    int u= static_cast<int>( floor( elevation_normalized/ interval_elevation ));
    if ( u == -1){ u= 0; } 
    if ( u == m_params.num_vertical_rings ){ --u;};

    const double interval_azimuth = static_cast<double>( 2*M_PI / m_params.num_points_ring);
    const double azimuth_normalized = t_azimuth + M_PI;
    int v= static_cast<int>( floor(azimuth_normalized / interval_azimuth));
    if ( v == -1){ v= 0; } 
    if ( v == m_params.num_points_ring){ --v;};

    vector<int> result;
    result.push_back(u);
    result.push_back(v);
    return result;
  }

  vector<int> SphericalDepthImage::mapCartesianCoordsInIndexImage(
      const Eigen::Vector3f & t_coords){
    Vector3f spherical_coords = SphericalDepthImage::directMappingFunc( t_coords);
    return  mapSphericalCoordsInIndexImage( spherical_coords.x(), spherical_coords.y()); 
  }
 

  Vector2f SphericalDepthImage::extimateMinMaxElevation( const PointNormalColor3fVectorCloud & cloud){
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

