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

  IntegralImage SphericalDepthImage::collectNormals(){
    discoverNormalsBoundaryIndexes();
    removePointsWithoutNormal();
    return computePointNormals();
    
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

  bool SphericalDepthImage::expandNormalBoundariesUp( DataPoint & t_starting_point, int & t_neighboors_count){

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




  bool SphericalDepthImage::expandNormalBoundariesDown( DataPoint & t_starting_point,int & t_neighboors_count){

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


  bool SphericalDepthImage::expandNormalBoundariesRight( DataPoint & t_starting_point,int & t_neighboors_count){

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


  bool SphericalDepthImage::expandNormalBoundariesLeft( DataPoint & t_starting_point,int & t_neighboors_count){

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

  void SphericalDepthImage::discoverNormalsBoundaryIndexes(){

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
               topFree =  expandNormalBoundariesUp( entry, neighboors_count);
               break;
              case(1):
               downFree =  expandNormalBoundariesDown( entry, neighboors_count);
               break;
              case(2):
               leftFree =  expandNormalBoundariesLeft( entry, neighboors_count);
               break;
              case(3):
               rightFree =  expandNormalBoundariesRight( entry, neighboors_count);
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

  IntegralImage SphericalDepthImage::computePointNormals(){
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
          //try if taking the column the result is different 
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
    return integ_img;
  }

  vector< vector< int>> SphericalDepthImage::findGoodClusterSeeds(){

    vector< vector< int>> goodSeeds;
    goodSeeds.reserve( m_cloud.size()/m_params.epsilon_c);

    vector<int> goodIndexes;
    goodIndexes.resize( m_cloud.size());
    for( unsigned int i =0; i< goodIndexes.size(); ++i){
      goodIndexes[i] = i;
    }
    //choose better values
    int alreadyClustered_hits_in_a_row = 0;
    const int limit_hits_in_a_row = 3;
    const int iterations_limit= m_cloud.size(); 
    int iteration_count= 0; 


    while ( iteration_count < iterations_limit
        and goodIndexes.size() > 0
        and countPointsNotClustered() >= m_params.epsilon_c){
      ++iteration_count;
      std::cout<<"iteration n:"<< iteration_count<< "\n";
      std::cout<<"not clustered num:"<< countPointsNotClustered() << "\n";

      if( alreadyClustered_hits_in_a_row > limit_hits_in_a_row){
        goodIndexes = fetchGoodSeedIndexes();
      }

      if( goodIndexes.size() >0){

        std::random_device rd;
        std::mt19937 gen( rd());
        std::uniform_int_distribution<> dis(0, goodIndexes.size() -1);
        int k  = goodIndexes[ dis( gen )];

        vector<int> indexImage_coords =  mapCartesianCoordsInIndexImage( m_cloud[k].coordinates());
        int counter_list_pos = 0;
        for( auto & elem: m_index_image[indexImage_coords[0]][indexImage_coords[1]]){
          if( elem.getIndexContainer() == k and 
              not elem.getIsClustered()){
            alreadyClustered_hits_in_a_row = 0;
            const int seed_row = indexImage_coords[0];
            const int seed_col = indexImage_coords[1] ;
            const int seed_list_pos = counter_list_pos ;
            bool validClusterFlag = discoverClusterBoundaryIndexes(seed_row, seed_col, seed_list_pos); 
            if( validClusterFlag ){
              vector<int> seed = { seed_row, seed_col, seed_list_pos};
              goodSeeds.push_back( seed);
            }
          }
          else if (elem.getIsClustered()){
            ++alreadyClustered_hits_in_a_row;
          }
          ++counter_list_pos;
        }
      }
    }
    return goodSeeds;
  }

  vector<vector< DataPoint>> SphericalDepthImage::flattenIndexImage(){
    vector<vector< DataPoint>> flattenedImage;
    flattenedImage.resize( m_params.num_vertical_rings);
    for ( auto & v : flattenedImage){
      v.resize( m_params.num_points_ring);
    }
   
    for (unsigned int row =0; row < m_index_image.size() ; ++row){
      for (unsigned int col=0; col < m_index_image[0].size(); ++col){
        DataPoint nearestPoint = DataPoint();
        if ( m_index_image[row][col].size() > 0 ){
          for ( auto& elem: m_index_image[row][col]){
            if ( nearestPoint.getIndexContainer() !=  -1){
              const Eigen::Vector3f nearest_cartesian_coords =
                m_cloud[nearestPoint.getIndexContainer()].coordinates();
              const Eigen::Vector3f nearest_spherical_coords =
                SphericalDepthImage::directMappingFunc( nearest_cartesian_coords);
              const Eigen::Vector3f current_cartesian_coords =
                m_cloud[elem.getIndexContainer()].coordinates();
              const Eigen::Vector3f current_spherical_coords =
                SphericalDepthImage::directMappingFunc( current_cartesian_coords);
              if ( nearest_spherical_coords.z() > current_spherical_coords.z() ){
                nearestPoint = elem;
              }
            }
            else{
              nearestPoint = elem;
            }
          }
        }
        flattenedImage[row][col] = nearestPoint;
      }
    }
    return flattenedImage;
  }
 

  /////////////////////


  vector<Matchable>  SphericalDepthImage::clusterizeCloud( IntegralImage & t_integ_img){


    vector< vector< int>> goodSeeds =  findGoodClusterSeeds();

    vector<Matchable> matchables;
    matchables.reserve( m_cloud.size()/m_params.epsilon_c);

    for( auto & seed: goodSeeds){
      const int seed_row = seed[0];
      const int seed_col = seed[1] ;
      const int seed_list_pos = seed[2];
         
      auto curr_seed = std::next( m_index_image[seed_row][seed_col].begin(), seed_list_pos);

      vector<int> boundaries = curr_seed->getBoundaries();

      const int row_min = boundaries[0];
      const int row_max = boundaries[1];
      const int col_min = boundaries[2];
      const int col_max = boundaries[3];
   
      IntegralCell cell = t_integ_img.getCellInsideBoundaries( row_min, row_max, col_min, col_max);

      Eigen::Vector3f  p_sum = cell.getPsum();
      Eigen::Matrix3f p_quad = cell.getPquad();
      int p_num = cell.getPnum();

      Eigen::Vector3f mu = p_sum/p_num;
      Eigen::Matrix3f S = p_quad/p_num;
      Eigen::Matrix3f sigma = S - mu * mu.transpose();
      Eigen::JacobiSVD<Eigen::Matrix3f> svd(sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);

      //remember (different from the paper) decreasing order for singularvalues (it follows that R is at reverse)
      Eigen::Matrix3f R = svd.matrixU();
      Eigen::Vector3f s = svd.singularValues();
      Eigen::Matrix3f Omega;
      Omega << s.x() ,0, 0,
            0, s.y(), 0,
            0, 0, s.z();

      PointNormalColor3fVectorCloud currPoints = fetchPointsInBoundaries( row_min, row_max, col_min, col_max);

      Line l  = Line( mu, R, Omega);
      const float eigenval_constr_line  =  l.computeEigenvalueConstraint();
      const float err_line = l.computeResidualError( currPoints);
      cout << "eigenval_constr_line " << eigenval_constr_line << "\n err_line "<< err_line <<"\n";

      if ( eigenval_constr_line < m_params.epsilon_l and
          err_line < m_params.epsilon_dl){
        cout<< "Is a line !\n";
        matchables.push_back( l);
      }
      else{
        Plane p = Plane( mu, R, Omega);
        const float eigenval_constr_plane =  p.computeEigenvalueConstraint();
        const float err_plane = p.computeResidualError( currPoints);
        cout << "eigenval_constr_plane " << eigenval_constr_plane<< "\n err_plane"<< err_plane<<"\n";

        if ( eigenval_constr_plane < m_params.epsilon_p and
            err_plane< m_params.epsilon_dp){
          cout<< "Is a plane !\n";
          matchables.push_back( p);
        }
      }
    } 
    return matchables;
  }


  PointNormalColor3fVectorCloud SphericalDepthImage::fetchPointsInBoundaries( 
      const int t_rowMin,const int t_rowMax,const int t_colMin,const int t_colMax){
    PointNormalColor3fVectorCloud pointsInside;
    pointsInside.reserve( m_cloud.size());

    for (int row =t_rowMin; row <= t_rowMax ; ++row){
      for (int col=t_colMin;  col <= t_colMax ; ++col){
        list<DataPoint> listPoints = getListDataPointsAt( row, col);
        for ( auto & e: listPoints ){
          pointsInside.push_back( m_cloud[e.getIndexContainer()] );
        }
      }
    }
    return pointsInside;
  }

  vector<int> SphericalDepthImage::fetchGoodSeedIndexes(){
    vector<int> goodIndexes;
    goodIndexes.reserve( m_cloud.size());

    for (unsigned int row =0; row < m_index_image.size() ; ++row){
      for (unsigned int col=0; col < m_index_image[0].size(); ++col){
        for ( auto& entry : m_index_image[row][col]){
          if( not entry.getIsClustered()){
            goodIndexes.push_back( entry.getIndexContainer());
          }
        }
      }
    }
    return goodIndexes;
  }

  int SphericalDepthImage::countPointsNotClustered(){
    int not_clustered_points = 0;
    for (unsigned int row =0; row <m_index_image.size() ; ++row){
      for (unsigned int col=0; col <m_index_image[0].size(); ++col){
        for ( auto& entry : m_index_image[row][col]){
          if( not entry.getIsClustered()){
            ++not_clustered_points;
          }
        }
      }
    }
    return not_clustered_points;
  }


  int SphericalDepthImage::countPointsIndexImage(){
    int points= 0;
    for (unsigned int row =0; row <m_index_image.size() ; ++row){
      for (unsigned int col=0; col <m_index_image[0].size(); ++col){
        for ( auto & elem: m_index_image[row][col]){
          ++points;
        }
      }
    }
    return points;
  }

  bool SphericalDepthImage::expandClusterBoundariesUp( DataPoint & t_seed_point,int & t_included_points_count){

    vector<int> curr_boundaries = t_seed_point.getBoundaries();

    const int old_row_min = curr_boundaries[0];
    const int old_row_max = curr_boundaries[1];
    const int old_col_min = curr_boundaries[2];
    const int old_col_max = curr_boundaries[3];
    const int new_row_min = old_row_min - 1;
    if( new_row_min < 0){
      return false;
    }

    bool hasExpanded= true;

    for( int c = old_col_min; c < old_col_max; ++c){
      for( auto & new_entry: m_index_image[new_row_min][c]){
        const Eigen::Vector3f new_cart_coords = 
          m_cloud[ new_entry.getIndexContainer()].coordinates();
        const Eigen::Vector3f new_normal = 
          m_cloud[ new_entry.getIndexContainer()].normal();
        for( auto & old_entry: m_index_image[old_row_min][c]){
          const Eigen::Vector3f old_cart_coords = 
            m_cloud[ old_entry.getIndexContainer()].coordinates();
          const Eigen::Vector3f old_normal = 
            m_cloud[ old_entry.getIndexContainer()].normal();

          const float diff_cart =  (new_cart_coords - old_cart_coords).norm();
          const float diff_normal = 1 - (new_normal.transpose() * old_normal);

          if (new_entry.getIsClustered() or diff_cart > m_params.epsilon_d or diff_normal > m_params.epsilon_n){
            hasExpanded= false;
          }
        }
      }
    }
       
    if (hasExpanded){
      vector<int> new_boundaries = { new_row_min, old_row_max, old_col_min, old_col_max };
      t_seed_point.setBoundaries( new_boundaries);
      for( int c = old_col_min; c < old_col_max; ++c){
        for( auto & new_entry: m_index_image[new_row_min][c]){
          ++t_included_points_count;
        }
      }
    }
    return hasExpanded;
  }
  bool SphericalDepthImage::expandClusterBoundariesDown( DataPoint & t_seed_point,int & t_included_points_count){
    vector<int> curr_boundaries = t_seed_point.getBoundaries();

    const int old_row_min = curr_boundaries[0];
    const int old_row_max = curr_boundaries[1];
    const int old_col_min = curr_boundaries[2];
    const int old_col_max = curr_boundaries[3];
    const int new_row_max = old_row_max + 1;
    if( new_row_max >= m_index_image.size()){
      return false;
    }
      

    bool hasExpanded= true;
    for( int c = old_col_min; c < old_col_max; ++c){
      for( auto & new_entry: m_index_image[new_row_max][c]){
        const Eigen::Vector3f new_cart_coords = 
          m_cloud[ new_entry.getIndexContainer()].coordinates();
        const Eigen::Vector3f new_normal = 
          m_cloud[ new_entry.getIndexContainer()].normal();
        for( auto & old_entry: m_index_image[old_row_max][c]){
          const Eigen::Vector3f old_cart_coords = 
            m_cloud[ old_entry.getIndexContainer()].coordinates();
          const Eigen::Vector3f old_normal = 
            m_cloud[ old_entry.getIndexContainer()].normal();
       
          const float diff_cart =  (new_cart_coords - old_cart_coords).norm();
          const float diff_normal = 1 - (new_normal.transpose() * old_normal);

          if (new_entry.getIsClustered() or diff_cart > m_params.epsilon_d or diff_normal > m_params.epsilon_n){
            hasExpanded= false;
          }
        }
      }
    }
    if (hasExpanded){
      vector<int> new_boundaries = { old_row_min, new_row_max, old_col_min, old_col_max };
      t_seed_point.setBoundaries( new_boundaries);
      for( int c = old_col_min; c < old_col_max; ++c){
        for( auto & new_entry: m_index_image[new_row_max][c]){
          ++t_included_points_count;
        }
      }
    }
    return hasExpanded;
  }

  bool SphericalDepthImage::expandClusterBoundariesLeft( DataPoint & t_seed_point,int & t_included_points_count){

    vector<int> curr_boundaries = t_seed_point.getBoundaries();

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


    bool hasExpanded= true;
    for( int r = old_row_min; r < old_row_max; ++r){
      for( auto & new_entry: m_index_image[r][new_col_min]){
        const Eigen::Vector3f new_cart_coords = 
          m_cloud[ new_entry.getIndexContainer()].coordinates();
        const Eigen::Vector3f new_normal = 
          m_cloud[ new_entry.getIndexContainer()].normal();
        for( auto & old_entry: m_index_image[r][old_col_min]){
          const Eigen::Vector3f old_cart_coords = 
            m_cloud[ old_entry.getIndexContainer()].coordinates();
          const Eigen::Vector3f old_normal = 
            m_cloud[ old_entry.getIndexContainer()].normal();
       
          const float diff_cart =  (new_cart_coords - old_cart_coords).norm();
          const float diff_normal = 1 - (new_normal.transpose() * old_normal);

          if (new_entry.getIsClustered() or diff_cart > m_params.epsilon_d or diff_normal > m_params.epsilon_n){
            hasExpanded= false;
          }
        }
      }
    }
    if (hasExpanded){
      vector<int> new_boundaries = { old_row_min, old_row_max, new_col_min, old_col_max };
      t_seed_point.setBoundaries( new_boundaries);
      for( int r = old_row_min; r < old_row_max; ++r){
        for( auto & new_entry: m_index_image[r][new_col_min]){
          ++t_included_points_count;
        }
      }
    }
    return hasExpanded;
  }

  bool SphericalDepthImage::expandClusterBoundariesRight( DataPoint & t_seed_point,int & t_included_points_count){

    vector<int> curr_boundaries = t_seed_point.getBoundaries();
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

    bool hasExpanded= true;
    for( int r = old_row_min; r < old_row_max; ++r){
      for( auto & new_entry: m_index_image[r][new_col_max]){
        const Eigen::Vector3f new_cart_coords = 
          m_cloud[ new_entry.getIndexContainer()].coordinates();
        const Eigen::Vector3f new_normal = 
          m_cloud[ new_entry.getIndexContainer()].normal();
        for( auto & old_entry: m_index_image[r][old_col_max]){
          const Eigen::Vector3f old_cart_coords = 
            m_cloud[ old_entry.getIndexContainer()].coordinates();
          const Eigen::Vector3f old_normal = 
            m_cloud[ old_entry.getIndexContainer()].normal();

          const float diff_cart =  (new_cart_coords - old_cart_coords).norm();
          const float diff_normal = 1 - (new_normal.transpose() * old_normal);
          
          if (new_entry.getIsClustered() or diff_cart > m_params.epsilon_d or diff_normal > m_params.epsilon_n){
            hasExpanded= false;
          }
        }
      }
    }
    if (hasExpanded){
      vector<int> new_boundaries = { old_row_min, old_row_max, old_col_min, new_col_max };
      t_seed_point.setBoundaries( new_boundaries);

      for( int r = old_row_min; r < old_row_max; ++r){
        for( auto & new_entry: m_index_image[r][new_col_max]){
          ++t_included_points_count;
        }
      }
    }
    return hasExpanded;
  }

  bool SphericalDepthImage::discoverClusterBoundaryIndexes(const int t_seed_row, const int t_seed_col, const int t_seed_list_position) {
    
    vector<int> curr_boundaries = { t_seed_row, t_seed_row, t_seed_col, t_seed_col};

    cout<< "old_boundaries";
    cout<< curr_boundaries[0]<< " ";
    cout<< curr_boundaries[1]<< " ";
    cout<< curr_boundaries[2]<< " ";
    cout<< curr_boundaries[3]<< " ";
    cout << "\n";

    auto curr_seed= std::next( m_index_image[t_seed_row][t_seed_col].begin(), t_seed_list_position);
    curr_seed->setBoundaries( curr_boundaries);


    int num_included_points = 0;
    bool topFree = true;
    bool downFree = true;
    bool leftFree = true;
    bool rightFree = true;
    int direction_counter = 0;
    int iteration_counter= 0;
    int times_no_points_added_in_a_row = 0;
    int threshold_not_added_in_a_row = 8;
    int old_num_included_points = num_included_points;
    const int iteration_limit = m_index_image.size()+ m_index_image[0].size() ;
    while (
      (topFree or downFree or leftFree or rightFree) and
      iteration_counter < iteration_limit and
      times_no_points_added_in_a_row < threshold_not_added_in_a_row
      ){
        switch( direction_counter % 4){
          case(0):
            topFree = expandClusterBoundariesUp( *curr_seed, num_included_points);
            break;
          case(1):
            downFree = expandClusterBoundariesDown( *curr_seed, num_included_points);
            break;
          case(2):
            leftFree = expandClusterBoundariesLeft( *curr_seed, num_included_points);
            break;
          case(3):
            rightFree = expandClusterBoundariesRight( *curr_seed, num_included_points);
            break;
          }
        ++direction_counter;
        ++iteration_counter;
        if ( num_included_points == old_num_included_points){
          ++times_no_points_added_in_a_row;
        }else{
          times_no_points_added_in_a_row = 0;
        }
        old_num_included_points = num_included_points;
    }
    markClusteredPoints( *curr_seed);

    vector<int> new_boundaries  = curr_seed->getBoundaries();
    cout<< "new_boundaries ";
    cout<< new_boundaries[0]<< " ";
    cout<< new_boundaries[1]<< " ";
    cout<< new_boundaries[2]<< " ";
    cout<< new_boundaries[3]<< " ";
    cout << "\n";

    cout<< "num_included_points " << num_included_points << "\n";
    
    return num_included_points >=  m_params.epsilon_c;
  }


  void SphericalDepthImage::markClusteredPoints(DataPoint & t_seed_point){

    vector<int> boundaries = t_seed_point.getBoundaries();

    const int row_min = boundaries[0];
    const int row_max = boundaries[1];
    const int col_min = boundaries[2];
    const int col_max = boundaries[3];
    int xx = 0;

    for( int r = row_min; r <= row_max; ++r){
      for( int c = col_min; c <= col_max; ++c){
        for( auto & elem: m_index_image[r][c]){
          elem.setIsClustered( true);
          ++xx;
        }
      }
    }
  }

  vector<int> SphericalDepthImage::mapSphericalCoordsInIndexImage(
      const float t_azimuth, const float t_elevation ){

    const double interval_elevation = static_cast<double>(
        (m_max_elevation - m_min_elevation) / m_params.num_vertical_rings);
    const float elevation_normalized = t_elevation - m_min_elevation;
    int u= static_cast<int>( floor( elevation_normalized/ interval_elevation ));
    if ( u == -1){ ++u; } 
    if ( u == m_params.num_vertical_rings ){ --u;};

    const double interval_azimuth = static_cast<double>( 2*M_PI / m_params.num_points_ring);
    const double azimuth_normalized = t_azimuth + M_PI;
    int v= static_cast<int>( floor(azimuth_normalized / interval_azimuth));
    if ( v == -1){ ++v; } 
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


  RGBImage SphericalDepthImage::drawIndexImg(){
    RGBImage result_img;
    result_img.create( m_params.num_vertical_rings, m_params.num_points_ring);
    result_img = cv::Vec3b(253, 246, 227);
    

    std::vector<Vector3f, Eigen::aligned_allocator<Vector3f> >  colors;
    const int num_colors = 100;
    colors.resize(num_colors+1);
    for(size_t i=0; i < colors.size(); ++i) {
      colors[i]= Vector3f(
          227.f* float(i)/num_colors,
          246.f* (1 - float(i)/num_colors),
          253.f* float(i)/num_colors);
    }
    colors[num_colors] = Vector3f( 255.f,207.f,130.f);

    const float max_depth = 30;

    for (unsigned int row =0; row <m_index_image.size() ; ++row){
      for (unsigned int col=0; col <m_index_image[0].size(); ++col){
        int counter = 0;
        float cumulative_depth =0;
        for ( auto & elem: m_index_image[row][col]){
          Vector3f spherical  =directMappingFunc( m_cloud[ elem.getIndexContainer()].coordinates() );
          cumulative_depth+= spherical.z();
          ++counter;
        }
        int color_index = 0; 
        float avg_depth;
        if ( counter >0){
          avg_depth =  cumulative_depth/counter;
          if ( avg_depth > max_depth) {
            color_index = num_colors-1;
          }
          else{
            color_index = avg_depth * (num_colors-1) / max_depth;
          }
        }
        else{
          color_index= num_colors;
        }
        result_img.at<cv::Vec3b>( row,col) =
        cv::Vec3b(colors[color_index].x() ,colors[color_index].y() ,colors[color_index].z());
      }
    }
    return result_img;
  }

  RGBImage SphericalDepthImage::drawNormalsImg(){
    RGBImage result_img;
    result_img.create( m_params.num_vertical_rings, m_params.num_points_ring);
    result_img = cv::Vec3b(253, 246, 227);
 
    for (unsigned int row =0; row <m_index_image.size() ; ++row){
      for (unsigned int col=0; col <m_index_image[0].size(); ++col){
        int counter = 0;
        Vector3f cumulative_normal = Vector3f( 0.f,0.f,0.f);
        for ( auto & elem: m_index_image[row][col]){
          cumulative_normal += directMappingFunc( m_cloud[ elem.getIndexContainer()].normal() );
          ++counter;
        }
        Vector3f avg_normal;
        if ( counter >0){
          avg_normal=  cumulative_normal.normalized() ;
          result_img.at<cv::Vec3b>( row,col) =
            cv::Vec3b( 255.f*avg_normal.x(), 255.f* avg_normal.y(), 255.f* avg_normal.z());
            //cout << "normal values : "<< avg_normal.x() << " " << avg_normal.y() << " " << avg_normal.z() << " \n";
        }
        else{
          result_img.at<cv::Vec3b>( row,col) =
            cv::Vec3b( 255.f, 255.f, 255.f);
        }
      }
    }
    return result_img;
  }

  RGBImage SphericalDepthImage::drawClustersImg(){
    RGBImage result_img;
    result_img.create( m_params.num_vertical_rings, m_params.num_points_ring);
    result_img = cv::Vec3b(253, 246, 227);
 
    return result_img;
  }

}

