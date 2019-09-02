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

  void SphericalDepthImage::projectCloud(){
    if (m_cloud.size()>0){
      for (unsigned int i = 0; i<m_cloud.size(); ++i){
        vector<int> coords = mapCartesianCoordsInIndexImage( m_cloud[i].coordinates());
        DataPoint dp( i);
        if ( m_index_image[coords[0]][coords[1]].getIndexContainer() == -1){
          m_index_image[coords[0]][coords[1]] = dp;
        }
        else{
          Vector3f new_spherical_coords = MyMath::directMappingFunc(m_cloud[i].coordinates());
          Vector3f old_cartesian_coords = m_cloud[ m_index_image[coords[0]][coords[1]].getIndexContainer()].coordinates();
          Vector3f old_spherical_coords = MyMath::directMappingFunc( old_cartesian_coords );
          if ( new_spherical_coords.z() < old_spherical_coords.z()){
            m_index_image[coords[0]][coords[1]] = dp;
          }
        }
      }
    }
  };

  void SphericalDepthImage::unprojectCloud(){
    int new_index_container = 0;
    PointNormalColor3fVectorCloud unprojectedCloud;
    unprojectedCloud.reserve( m_cloud.size());
    for (unsigned int row =0; row <m_index_image.size() ; ++row){
      for (unsigned int col=0; col <m_index_image[0].size(); ++col){
        const int old_index =  m_index_image[row][col].getIndexContainer();
        if ( old_index != -1){
          unprojectedCloud.push_back( m_cloud[old_index]);
          m_index_image[row][col].setIndexContainer( new_index_container);
          ++ new_index_container;
        }
      }
    }
    m_cloud = unprojectedCloud;
  }

  void SphericalDepthImage::initializeIndexImage(){
    projectCloud();
    unprojectCloud();
  }

  void SphericalDepthImage::executeOperations(){
    removeFlatSurfaces();
    collectNormals();
  }


  void SphericalDepthImage::removeFlatSurfaces(){
    markVerticalPoints();
    removeNonVerticalPoints();
  }

  IntegralImage SphericalDepthImage::collectNormals(){
    discoverNormalsBoundaryIndexes();
    removePointsWithoutNormal();
    return computePointNormals();
    
 }
 void SphericalDepthImage::markVerticalPoints(){

    for (unsigned int col= 0; col<m_index_image[0].size(); ++col){
      for (int row = m_index_image.size() -1; row >= 0; --row){
        DataPoint curr_point =  m_index_image[row][col];
        if( not curr_point.getIsVertical() and curr_point.getIndexContainer() != -1){
          int num_points_falling_in_projection = 0;
          Vector3f ref_coords = m_cloud[ curr_point.getIndexContainer()].coordinates();
          vector< vector< int>> curr_indexes;
          for (int curr_row = row-1; curr_row >= 0; --curr_row){
            DataPoint other_point = m_index_image[curr_row][col];
            if( other_point.getIndexContainer() != -1){
              const float x_ref = ref_coords[0];
              const float y_ref = ref_coords[1];
              const Vector3f curr_coords = m_cloud[ other_point.getIndexContainer()].coordinates();
              const float x_curr = curr_coords[0];
              const float y_curr = curr_coords[1];
              const float distance = sqrt( pow( x_ref - x_curr, 2)+ pow( y_ref - y_curr, 2));
              if( distance < m_params.epsilon_radius){
                ++num_points_falling_in_projection;
                vector<int> curr_index;
                curr_index.reserve(2);
                curr_index.push_back(curr_row);
                curr_index.push_back(col);
                curr_indexes.push_back( curr_index);
              }
            }
          }
          if( num_points_falling_in_projection >= m_params.epsilon_times){
            for( auto & indexes: curr_indexes){
              m_index_image[indexes[0]][indexes[1]].setIsVertical( true);
            }
          }
        }
      }
    }
  }

  void SphericalDepthImage::removeNonVerticalPoints(){
    for (unsigned int row =0; row <m_index_image.size() ; ++row){
      for (unsigned int col=0; col <m_index_image[0].size(); ++col){
        DataPoint & curr_point = m_index_image[row][col];
        if( not curr_point.getIsVertical() and curr_point.getIndexContainer() != -1){
          curr_point.setIndexContainer( -1);
        }
      }
    }
    unprojectCloud();
  }

  void SphericalDepthImage::removePointsWithoutNormal(){
    for (unsigned int row =0; row <m_index_image.size() ; ++row){
      for (unsigned int col=0; col <m_index_image[0].size(); ++col){
        DataPoint & curr_point = m_index_image[row][col];
        if(not curr_point.getHasNormal() and curr_point.getIndexContainer() != -1){
          curr_point.setIndexContainer( -1);
        }
      }
    }
    unprojectCloud();
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
      MyMath::directMappingFunc( starting_cartesian_coords);
    bool hasExpanded= true;

    int current_added_neighboors = 0;
    for( unsigned int c = old_col_min; c < old_col_max; ++c){
      DataPoint other_point = m_index_image[new_row_min][c];
      if ( other_point.getIndexContainer() != -1){
        const Eigen::Vector3f other_cartesian_coords =
          m_cloud[ other_point.getIndexContainer()].coordinates();
        const Eigen::Vector3f other_spherical_coords =
          MyMath::directMappingFunc( other_cartesian_coords);

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
      MyMath::directMappingFunc( starting_cartesian_coords);
    bool hasExpanded= true;

    int current_added_neighboors = 0;
    for( unsigned int c = old_col_min; c < old_col_max; ++c){
      DataPoint other_point = m_index_image[new_row_max][c];
      if ( other_point.getIndexContainer() != -1){
        const Eigen::Vector3f other_cartesian_coords =
          m_cloud[ other_point.getIndexContainer()].coordinates();
        const Eigen::Vector3f other_spherical_coords =
          MyMath::directMappingFunc( other_cartesian_coords);

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
      MyMath::directMappingFunc( starting_cartesian_coords);
    bool hasExpanded= true;

    int current_added_neighboors = 0;
    for( unsigned int r = old_row_min; r < old_row_max; ++r){
      DataPoint other_point = m_index_image[r][new_col_min];
      if ( other_point.getIndexContainer() != -1){
        const Eigen::Vector3f other_cartesian_coords =
          m_cloud[ other_point.getIndexContainer()].coordinates();
        const Eigen::Vector3f other_spherical_coords =
          MyMath::directMappingFunc( other_cartesian_coords);

        if ( fabs( starting_spherical_coords.z() - other_spherical_coords.z()) >
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
      MyMath::directMappingFunc( starting_cartesian_coords);
    bool hasExpanded= true;
    int current_added_neighboors = 0;

    for( unsigned int r = old_row_min; r < old_row_max; ++r){
      DataPoint other_point = m_index_image[r][new_col_max];
      if ( other_point.getIndexContainer() != -1){
   
        const Eigen::Vector3f other_cartesian_coords =
          m_cloud[ other_point.getIndexContainer()].coordinates();
        const Eigen::Vector3f other_spherical_coords =
          MyMath::directMappingFunc( other_cartesian_coords);

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
        DataPoint & curr_point = m_index_image[row][col];
        if ( curr_point.getIndexContainer() != -1){
          vector<int> curr_boundaries = { row, row, col, col};
          curr_point.setBoundaries( curr_boundaries);
          int up_neighboors_count= 0;
          int down_neighboors_count= 0;
          int left_neighboors_count= 0;
          int right_neighboors_count= 0;
          int total_neighboors_count = up_neighboors_count + down_neighboors_count+
            left_neighboors_count + right_neighboors_count;
          bool topFree = true;
          bool downFree = true;
          bool leftFree = true;
          bool rightFree = true;
          int direction_counter = 0;
          int iteration_counter= 0;
          const int iteration_limit = m_params.min_neighboors_for_normal*4;
          while (
              total_neighboors_count <  m_params.min_neighboors_for_normal and
              (topFree or downFree or leftFree or rightFree) and
              iteration_counter < iteration_limit
              ){
            switch( direction_counter % 4){
              case(0):
               topFree =  expandNormalBoundariesUp( curr_point, up_neighboors_count);
               break;
              case(1):
               downFree =  expandNormalBoundariesDown( curr_point, down_neighboors_count);
               break;
              case(2):
               leftFree =  expandNormalBoundariesLeft( curr_point, left_neighboors_count);
               break;
              case(3):
               rightFree =  expandNormalBoundariesRight( curr_point, right_neighboors_count);
               break;
            }
            ++direction_counter;
            ++iteration_counter;
            total_neighboors_count = up_neighboors_count + down_neighboors_count+
              left_neighboors_count + right_neighboors_count;
          
          }
          const bool hasExpandedInEveryDirection =
            up_neighboors_count>0 and down_neighboors_count>0 and
            left_neighboors_count>0 and right_neighboors_count>0;
          cout<< "hasExpandedInEveryDirection : " << hasExpandedInEveryDirection<< "\n";
          cout<< "total_neighboors_count : " << total_neighboors_count<< "\n";
          cout<< "up_neighboors_count : " << up_neighboors_count<< "\n";
          cout<< "down_neighboors_count : " << down_neighboors_count<< "\n";
          cout<< "left_neighboors_count : " << left_neighboors_count<< "\n";
          cout<< "right_neighboors_count : " << right_neighboors_count<< "\n";


          // TODO analyze why the normal expansion is never in every direction,
          // there is at least one direction where it does not find neighboors
          const bool hasNormal = (total_neighboors_count >=  m_params.min_neighboors_for_normal);
           // and hasExpandedInEveryDirection;
          curr_point.setHasNormal( hasNormal );
        }
      }
    }
  }

  IntegralImage SphericalDepthImage::computePointNormals(){
    
    for (int row =0; row <m_index_image.size() ; ++row){
      for (int col=0; col <m_index_image[0].size(); ++col){
        DataPoint & curr_point = m_index_image[row][col];
        const int index_inContainer = curr_point.getIndexContainer();
        if ( index_inContainer != -1){
          int min_row = curr_point.getBoundaries()[0];
          int max_row = curr_point.getBoundaries()[1];
          int min_col = curr_point.getBoundaries()[2];
          int max_col = curr_point.getBoundaries()[3];


          const vector<Eigen::Vector3f> neighboors =  fetchPointsInBoundaries( min_row, max_row, min_col, max_col);
          const Eigen::Vector3f mu  = MyMath::computeMuGaussian( neighboors);
          const Eigen::Matrix3f sigma = MyMath::computeSigmaGaussian( neighboors, mu);
          Eigen::JacobiSVD<Eigen::Matrix3f> svd(sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);

          Eigen::Matrix3f R = svd.matrixU();
          //try if taking the column the result is different 
          Eigen::Vector3f smallestEigenVec= R.col(2);

          Eigen::Vector3f p_coords =  m_cloud[ index_inContainer].coordinates();
          if ( (p_coords + smallestEigenVec).norm() >  p_coords.norm() ){
            m_cloud[ index_inContainer].normal() =  - smallestEigenVec.normalized();
          }
          else{
            m_cloud[ index_inContainer].normal() = smallestEigenVec.normalized();
          }
        }
      }
    }
    // stub integ image only to respect the signature of the function
    IntegralImage integ_img = IntegralImage( m_cloud, m_index_image);
    return integ_img;

  }


  IntegralImage SphericalDepthImage::computePointNormalsDeprecated(){
    IntegralImage integ_img = IntegralImage( m_cloud, m_index_image);
    integ_img.buildIntegMatrix();
    
    for (int row =0; row <m_index_image.size() ; ++row){
      for (int col=0; col <m_index_image[0].size(); ++col){
        DataPoint & curr_point = m_index_image[row][col];
        const int index_inContainer = curr_point.getIndexContainer();
        if ( index_inContainer != -1){
          int min_row = curr_point.getBoundaries()[0];
          int max_row = curr_point.getBoundaries()[1];
          int min_col = curr_point.getBoundaries()[2];
          int max_col = curr_point.getBoundaries()[3];

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

          Eigen::Vector3f p_coords =  m_cloud[ index_inContainer].coordinates();
          if ( (p_coords + smallestEigenVec).norm() >  p_coords.norm() ){
            m_cloud[ index_inContainer].normal() =  - smallestEigenVec.normalized();
          }
          else{
            m_cloud[ index_inContainer].normal() = smallestEigenVec.normalized();
          }
        }
      }
    }
    return integ_img;
  }


  PointNormalColor3fVectorCloud SphericalDepthImage::fetchPoints(const vector<int> & indexes){
    PointNormalColor3fVectorCloud points;
    points.reserve( indexes.size());
    for ( auto & index: indexes){
      points.push_back( m_cloud[ index] );
    }
    return points;
  }

  vector< Vector3f>  SphericalDepthImage::fetchPointsInBoundaries(
          const int t_rowMin,const int t_rowMax,const int t_colMin,const int t_colMax){
    vector< Vector3f> pointsInside; 
    pointsInside.reserve( m_cloud.size());
    for (int row =t_rowMin; row <= t_rowMax ; ++row){
      for (int col=t_colMin;  col <= t_colMax ; ++col){
        DataPoint p = m_index_image[row][col];  
        if (p.getIndexContainer() != -1){
          PointNormalColor3f point = m_cloud[ p.getIndexContainer()];
          pointsInside.push_back( point.coordinates());
        }
      }
    }
    return pointsInside;
  }

  vector<Matchable>  SphericalDepthImage::clusterizeCloud(){
      vector<Matchable> matchables;
      m_clusterer = Clusterer(m_cloud, m_index_image, m_params);
      vector<cluster> clusters =  m_clusterer.findClusters();

      for ( auto &c: clusters){

        Eigen::JacobiSVD<Eigen::Matrix3f> svd(c.sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);
        //remember (different from the paper) decreasing order for singularvalues (it follows that R is at reverse)
        Eigen::Matrix3f R = svd.matrixU();
        Eigen::Vector3f s = svd.singularValues();
        Eigen::Matrix3f Omega;
        Omega << s.x() ,0, 0,
            0, s.y(), 0,
            0, 0, s.z();

        PointNormalColor3fVectorCloud currPoints = fetchPoints( c.indexes);

        Line l  = Line( c.mu, R, Omega);
        const float eigenval_constr_line  =  l.computeEigenvalueConstraint();
        const float err_line = l.computeResidualError( currPoints);
        cout << "eigenval_constr_line " << eigenval_constr_line << "\n err_line "<< err_line <<"\n";
        if ( eigenval_constr_line < m_params.epsilon_l and
            err_line < m_params.epsilon_dl){
          cout<< "Is a line !\n";
          matchables.push_back( l);
        }
        else{
          Plane p = Plane( c.mu, R, Omega);
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
    Vector3f spherical_coords = MyMath::directMappingFunc( t_coords);
    return  mapSphericalCoordsInIndexImage( spherical_coords.x(), spherical_coords.y()); 
  }
 

  Vector2f SphericalDepthImage::extimateMinMaxElevation( const PointNormalColor3fVectorCloud & cloud){
    float min_elevation = 0;
    float max_elevation = 0;
    if (cloud.size()>0){
      Vector3f initialSphericalCoords = MyMath::directMappingFunc( cloud[0].coordinates());
      min_elevation = initialSphericalCoords.y();
      max_elevation = initialSphericalCoords.y();
      Vector3f curr_sphericalCoords;
      for ( auto & p: cloud){
        curr_sphericalCoords = MyMath::directMappingFunc( p.coordinates());
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

  int SphericalDepthImage::countPointsValid(){
    int points= 0;
    for (unsigned int row =0; row <m_index_image.size() ; ++row){
      for (unsigned int col=0; col <m_index_image[0].size(); ++col){
        DataPoint curr_point =   m_index_image[row][col];
        if( curr_point.getIndexContainer() != -1 ){
          ++points;
        }
      }
    }
    return points;
  }

  
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
          246.f* float(i)/num_colors,
          253.f* float(i)/num_colors);
    }
    colors[num_colors] = Vector3f( 255.f, 255.f, 255.f);

    const float max_depth = 50;

    for (unsigned int row =0; row <m_index_image.size() ; ++row){
      for (unsigned int col=0; col <m_index_image[0].size(); ++col){
        DataPoint curr_point =   m_index_image[row][col];
        int color_index = 0; 
        if( curr_point.getIndexContainer() != -1 ){
          Vector3f spherical = MyMath::directMappingFunc( m_cloud[ curr_point.getIndexContainer()].coordinates() );
          const float depth= spherical.z();
          if ( depth> max_depth) {
            color_index = num_colors-1;
          }
          else{
            color_index = depth * (num_colors-1) / max_depth;
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
        DataPoint curr_point =   m_index_image[row][col];
        if( curr_point.getIndexContainer() != -1 ){
          const  Vector3f normal =  m_cloud[ curr_point.getIndexContainer()].normal() ;
          result_img.at<cv::Vec3b>( row,col) =
            cv::Vec3b( 255.f*normal.x(), 255.f* normal.y(), 255.f* normal.z());
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

  vector<RGBImage> SphericalDepthImage::drawImgsClusterer(){
    vector<RGBImage> images;
    images.reserve(2);
    images.push_back( m_clusterer.drawPathImg());
    images.push_back( m_clusterer.drawBlurredNormalsImg());
    return images;
  }

  // todo   
  RGBImage SphericalDepthImage::drawClustersImg(){
    RGBImage result_img;
    result_img.create( m_params.num_vertical_rings, m_params.num_points_ring);
    result_img = cv::Vec3b(253, 246, 227);
 
    return result_img;
  }

}

