#include "Clusterer.hpp"

namespace Loam{

  Clusterer::Clusterer( 
         const  PointNormalColor3fVectorCloud & t_cloud,
         vector<vector<DataPoint>> t_index_image,
         const sphericalImage_params t_params):
    m_cloud(t_cloud),
    m_index_image( t_index_image),
    m_params(t_params){

    m_pathMatrix = populatePathMatrix(
      t_cloud, t_index_image,t_params);

 }
         
  vector<vector<pathCell>>  Clusterer::populatePathMatrix(
      const  PointNormalColor3fVectorCloud & t_cloud,
      vector<vector<DataPoint>> t_index_image,
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

    vector<vector< Eigen::Vector3f>>  m_blurredNormalsMatrix;
    m_blurredNormalsMatrix.resize(m_params.num_vertical_rings);
    for ( auto & vec: m_blurredNormalsMatrix ){
      vec.resize( m_params.num_points_ring);
    }
 
    const int  blur_extension = 2;
    for (int row =0; row < m_pathMatrix.size() ; ++row){
      for (int col=0; col < m_pathMatrix[0].size(); ++col){
        int num_near_normals= 0;
        Eigen::Vector3f cumulative_normal = Eigen::Vector3f::Zero();
        cout<< " current  row  col:"<< row << " " << col<< "\n";
       
        for (int near_row= row - blur_extension; near_row <= row + blur_extension; ++near_row){
          for (int near_col= col - blur_extension; near_col <= col + blur_extension; ++near_col){
            if ( near_row >= 0 and near_row < m_params.num_vertical_rings and
              near_col >= 0 and near_col < m_params.num_points_ring){
              cout<< " first if passed \n";
              if ( m_pathMatrix[near_row][near_col].normal.norm() > 1e-3 ){
                cout<< " second if passed \n";
                cout<< " current near row  and col:"<< near_row << " " << near_col<< "\n";
                cout<< " current normal:\n"<< m_pathMatrix[near_row][near_col].normal<< "\n";
                //cout<< " current  Norm:\n"<< m_pathMatrix[near_row][near_col].normal.norm()<< "\n";

                cumulative_normal += m_pathMatrix[near_row][near_col].normal;
                ++num_near_normals;
              }
            }
          }
        }
        if (num_near_normals >0){
          m_blurredNormalsMatrix[row][col] = (cumulative_normal/ num_near_normals).normalized();
          cout<< "END passed\n";
          cout<< "cumulative_normal:\n"<< cumulative_normal  << "\n";
          cout<< "num_near_normals:"<< num_near_normals<< "\n";
          cout<< "resulting :\n"<< m_blurredNormalsMatrix[row][col]  << "\n";
        }
        else{
          cout<< "END discarded\n";
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
    while( not  allCellsChosen ){
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
        if (! m_pathMatrix[row][col].hasBeenChoosen){
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
          not m_pathMatrix[coord.row][coord.col].hasBeenChoosen){
        cells.push_back( m_pathMatrix[coord.row][coord.col]);
      }
    }
    return cells;
  }

  cluster Clusterer::computeCluster(const matrixCoords & t_seed_coords){
    stack<pathCell> cellStack;
    cellStack.push( m_pathMatrix[t_seed_coords.row][t_seed_coords.col]);
    cluster c;
    int numPointsCluster =0;
    Eigen::Vector3f cumulative_mu;
    Eigen::Matrix3f cumulative_S;
    vector<matrixCoords> matrixIndexes;
//    //find a good number to preserve space
//    while( not cellStack.empty()){
//      pathCell currCell = cellStack.top();
//      vector<pathCell> neighboors=  findNeighboors( currCell);
//      ++numPointsCluster;
//      cellStack.pop();
//      DataPoint currDataPoint = m_index_image[currCell.matCoords.row][currCell.matCoords.col];
//      const Eigen::Vector3f currCart_coords= m_cloud[ currDataPoint.getIndexContainer()].coordinates();
//      const Eigen::Vector3f currNormal= m_cloud[ currDataPoint.getIndexContainer()].normal();
//
//      for ( auto & otherCell: neighboors){
//        if( abs( currCell.depth - otherCell.depth) < m_params.depth_differential_threshold){
//          DataPoint otherDataPoint = m_index_image[otherCell.matCoords.row][otherCell.matCoords.col];
//          const Eigen::Vector3f otherCart_coords=
//            m_cloud[ otherDataPoint.getIndexContainer()].coordinates();
//          const Eigen::Vector3f otherNormal= m_cloud[ otherDataPoint.getIndexContainer()].normal();
//          const float diff_cartesian = ( currCart_coords - otherCart_coords).norm();
//          const float diff_normal= 1- ( currNormal.transpose() * otherNormal);
//          if ( diff_cartesian < m_params.epsilon_d and diff_normal < m_params.epsilon_n){
//            cellStack.push( otherCell);
//          }
//        }
//      }
//
//      cumulative_mu += currCart_coords;
//      cumulative_S += currCart_coords * currCart_coords.transpose();
//      indexes.push_back(currDataPoint.getIndexContainer());
//    }

    while( not cellStack.empty()){
      pathCell currCell = cellStack.top();
      vector<pathCell> neighboors=  findNeighboors( currCell);
      ++numPointsCluster;
      cellStack.pop();
      for ( auto & otherCell: neighboors){
        const float diff_normal= 1- ( currCell.normal.transpose() * otherCell.normal);
        if ( diff_normal < m_params.epsilon_n){
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
      for ( auto & matIndex: matrixIndexes){
        m_pathMatrix[matIndex.row][matIndex.col].hasBeenChoosen = true;
      }
      Eigen::Matrix3f S = cumulative_S/numPointsCluster;
      c.mu = cumulative_mu/ numPointsCluster;
      c.sigma = S - c.mu*c.mu.transpose();
    }
    else{
      m_pathMatrix[t_seed_coords.row][t_seed_coords.col].hasBeenChoosen = true;
    }
    return c;
  }


  RGBImage Clusterer::drawPathImg(){
    RGBImage result_img;
    result_img.create( m_params.num_vertical_rings, m_params.num_points_ring);
    result_img = cv::Vec3b(253, 246, 227);

    std::vector<Vector3f, Eigen::aligned_allocator<Vector3f> >  colors;
    const int num_colors = 100;
    colors.resize(num_colors);
    for(size_t i=0; i < colors.size(); ++i) {
      colors[i]= Vector3f(
          253.f*  float(i)/num_colors,
          246.f*  float(i)/num_colors,
          227.f*  float(i)/num_colors);
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
    result_img = cv::Vec3b(253, 246, 227);

    for (unsigned int row =0; row <m_blurredNormalsMatrix.size() ; ++row){
      for (unsigned int col=0; col <m_blurredNormalsMatrix[0].size(); ++col){
        Eigen::Vector3f  normal=  m_blurredNormalsMatrix[row][col];
        result_img.at<cv::Vec3b>( row,col) =
          cv::Vec3b( 255.f*normal.x(), 255.f* normal.y(), 255.f* normal.z());
      }
    }
    return result_img;
  }

}


