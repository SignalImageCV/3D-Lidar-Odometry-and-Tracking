#include "Clusterer.hpp"

namespace Loam{

  Clusterer::Clusterer( 
         const  PointNormalColor3fVectorCloud & t_cloud,
         const vector<vector<DataPoint>> & t_index_image,
         const sphericalImage_params t_params):
    m_cloud(t_cloud),
    m_index_image( t_index_image),
    m_params(t_params){

    m_pathMatrix = populatePathMatrix(
      t_cloud, t_index_image,t_params);

    m_blurredNormalsMatrix.resize(m_params.num_vertical_rings);
    for ( auto & external_vec: m_blurredNormalsMatrix ){
      external_vec.resize( m_params.num_points_ring);
      for ( auto & elem : external_vec){
        elem = Vector3f::Zero();
      }
    }
  }
         
  vector<vector<pathCell>>  Clusterer::populatePathMatrix(
      const  PointNormalColor3fVectorCloud & t_cloud,
      const vector<vector<DataPoint>> & t_index_image,
      const sphericalImage_params t_params){
 
    vector<vector<pathCell>> pathMatrix;
    pathMatrix.resize(t_params.num_vertical_rings);
    for ( auto & vec: pathMatrix){
      vec.resize( t_params.num_points_ring);
    }

    for (unsigned int row =0; row < t_index_image.size() ; ++row){
      for (unsigned int col=0; col < t_index_image[0].size(); ++col){
        DataPoint curr_point = t_index_image[row][col];
        if ( curr_point.getIndexContainer() != -1){
          const Eigen::Vector3f cart_coords =
            t_cloud[curr_point.getIndexContainer()].coordinates();
          const Eigen::Vector3f sph_coords =
            MyMath::directMappingFunc( cart_coords);
          pathMatrix[row][col].depth = sph_coords.z();
          pathMatrix[row][col].normal = t_cloud[curr_point.getIndexContainer()].normal();
        }
        pathMatrix[row][col].matCoords = matrixCoords( row,col);
      }
    }

    return  pathMatrix;
  }

  void Clusterer::blurNormals(){
    
     

    const int  blur_extension = 4;
    for (int row =0; row < m_pathMatrix.size() ; ++row){
      for (int col=0; col < m_pathMatrix[0].size(); ++col){
        int num_near_normals= 0;
        Eigen::Vector3f cumulative_normal = Eigen::Vector3f::Zero();
       
        for (int near_row= row - blur_extension; near_row <= row + blur_extension; ++near_row){
          for (int near_col= col - blur_extension; near_col <= col + blur_extension; ++near_col){
            if ( near_row >= 0 and near_row < m_params.num_vertical_rings and
              near_col >= 0 and near_col < m_params.num_points_ring){
              if ( m_pathMatrix[near_row][near_col].normal.norm() > 1e-3 ){
                cumulative_normal += m_pathMatrix[near_row][near_col].normal;
                ++num_near_normals;
              }
            }
          }
        }
        if (num_near_normals >0){
          m_blurredNormalsMatrix[row][col] = (cumulative_normal/ num_near_normals).normalized();
        }
        else{
          m_blurredNormalsMatrix[row][col] = Eigen::Vector3f::Zero();
        }
      }
    }
  }

  vector<cluster> Clusterer::findClusters(){

    blurNormals();

    vector<cluster> clusters;
    //find a good number to preserve space

    bool allCellsChosen = false;
    const int max_iterations = m_pathMatrix.size() * m_pathMatrix[0].size();
    int counter = 0;
    while( not  allCellsChosen and counter < max_iterations ){
      ++counter;
      matrixCoords coords =  findSeed();
      if( coords.row == -1 or coords.col == -1){
        allCellsChosen = true;
      }
      else{
        cluster c = computeCluster( coords);
        if ( c.indexes.size() >= m_params.epsilon_c){
          clusters.push_back( c);
        }
      }
    }
    return clusters;
  }

  matrixCoords Clusterer::findSeed(){

    matrixCoords mC;
    bool found = false;
    for (unsigned int row =0; row < m_pathMatrix.size() ; ++row){
      for (unsigned int col=0; col < m_pathMatrix[0].size(); ++col){
        if (not m_pathMatrix[row][col].hasBeenChosen
            and m_pathMatrix[row][col].depth < 1e+4 
            ){
          found = true;
          mC.row = row;
          mC.col = col;
          break;
          break;
        }
      }
    }
    if (! found){
      mC.row = -1;
      mC.col = -1;
    }
    return mC;
  }

  vector<pathCell> Clusterer::findNeighboors(pathCell & t_pathCell){
    matrixCoords upLeft( t_pathCell.matCoords.row -1, t_pathCell.matCoords.col -1);
    matrixCoords up( t_pathCell.matCoords.row -1, t_pathCell.matCoords.col);
    matrixCoords upRight( t_pathCell.matCoords.row -1, t_pathCell.matCoords.col +1);
    matrixCoords left( t_pathCell.matCoords.row , t_pathCell.matCoords.col -1);
    matrixCoords right( t_pathCell.matCoords.row , t_pathCell.matCoords.col +1);
    matrixCoords downLeft( t_pathCell.matCoords.row +1, t_pathCell.matCoords.col -1);
    matrixCoords down( t_pathCell.matCoords.row +1, t_pathCell.matCoords.col );
    matrixCoords downRight( t_pathCell.matCoords.row +1, t_pathCell.matCoords.col +1);

    vector<matrixCoords> coords = {
      upLeft, up, upRight, left, right, downLeft, down, downRight
    };

    vector<pathCell> cells;
    cells.reserve( coords.size());

    for( auto& coord: coords){
      if ( coord.row >= 0 and coord.row < m_params.num_vertical_rings and
          coord.col >= 0 and coord.col < m_params.num_points_ring and
          not m_pathMatrix[coord.row][coord.col].hasBeenChosen and
          m_pathMatrix[coord.row][coord.col].depth < 1e+4 ){
        cells.push_back( m_pathMatrix[coord.row][coord.col]);
      }
    }
    return cells;
  }

  cluster Clusterer::computeCluster(const matrixCoords & t_seed_coords){
    stack<pathCell> cellStack;
    m_pathMatrix[t_seed_coords.row][t_seed_coords.col].hasBeenChosen = true;
    cellStack.push( m_pathMatrix[t_seed_coords.row][t_seed_coords.col]);
    cluster c;
    int numPointsCluster =0;
    Eigen::Vector3f cumulative_mu= Eigen::Vector3f::Zero();
    Eigen::Matrix3f cumulative_S = Eigen::Matrix3f::Zero();
    vector<matrixCoords> matrixIndexes;
    while( not cellStack.empty()){
      pathCell currCell = cellStack.top();
      vector<pathCell> neighboors=  findNeighboors( currCell);
      ++numPointsCluster;
      cellStack.pop();
      for ( auto & otherCell: neighboors){
        const float diff_normal= 1- ( currCell.normal.transpose() * otherCell.normal);
        if ( diff_normal < m_params.epsilon_n){
            m_pathMatrix[otherCell.matCoords.row][otherCell.matCoords.col].hasBeenChosen = true;
            cellStack.push( otherCell);
        }
      }
      DataPoint currDataPoint = m_index_image[currCell.matCoords.row][currCell.matCoords.col];
      const Eigen::Vector3f currCart_coords= m_cloud[ currDataPoint.getIndexContainer()].coordinates();
      cumulative_mu += currCart_coords;
      cumulative_S += currCart_coords * currCart_coords.transpose();
      c.indexes.push_back(currDataPoint.getIndexContainer());
      matrixIndexes.push_back( currCell.matCoords);
    }

    if ( c.indexes.size() >= m_params.epsilon_c){
      Eigen::Matrix3f S = cumulative_S/numPointsCluster;
      c.mu = cumulative_mu/ numPointsCluster;
      c.sigma = S - c.mu*c.mu.transpose();
    }
    return c;
  }


  RGBImage Clusterer::drawPathImg(){
    RGBImage result_img;
    result_img.create( m_params.num_vertical_rings, m_params.num_points_ring);
    result_img = cv::Vec3b(255, 255, 255);

    std::vector<Vector3f, Eigen::aligned_allocator<Vector3f> >  colors;
    const int num_colors = 100;
    colors.resize(num_colors);
    for(size_t i=0; i < colors.size(); ++i) {
      colors[i]= Vector3f(
          255.f*  float(i)/num_colors,
          255.f*  float(i)/num_colors,
          255.f*  float(i)/num_colors);
    }

    const float max_depth =100;
    for (unsigned int row =0; row <m_pathMatrix.size() ; ++row){
      for (unsigned int col=0; col <m_pathMatrix[0].size(); ++col){
        float depth= m_pathMatrix[row][col].depth;

        int color_index = 0; 
        if ( depth > max_depth) {
          color_index = num_colors-2;
        }
        else{
          color_index = depth * (num_colors-1) / max_depth;
        }
        result_img.at<cv::Vec3b>( row,col) =
        cv::Vec3b(colors[color_index].x() ,colors[color_index].y() ,colors[color_index].z());
      }
    }
    return result_img;
  }


  RGBImage Clusterer::drawBlurredNormalsImg(){
    RGBImage result_img;
    result_img.create( m_params.num_vertical_rings, m_params.num_points_ring);
    result_img = cv::Vec3b(255, 255, 255);

    for (unsigned int row =0; row <m_blurredNormalsMatrix.size() ; ++row){
      for (unsigned int col=0; col <m_blurredNormalsMatrix[0].size(); ++col){
        Eigen::Vector3f  normal=  m_blurredNormalsMatrix[row][col];
        if ( normal.norm() > 1e-3){
          result_img.at<cv::Vec3b>( row,col) =
            cv::Vec3b(
              (255.f - 255.f*normal.x()),
              (255.f - 255.f* normal.y()),
              (255.f - 255.f* normal.z()));
        }
        else{
          result_img.at<cv::Vec3b>( row,col) =
            cv::Vec3b(155, 5, 55);
        }
      }
    }
    return result_img;
  }

}


