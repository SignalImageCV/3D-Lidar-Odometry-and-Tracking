#include "IntegralImage.hpp"

namespace Loam{

  IntegralImage::IntegralImage(
         const  PointNormalColor3fVectorCloud & t_cloud,
         vector<vector<list<DataPoint>>> t_index_image):
    IntegralImageInterface(),
    m_cloud( t_cloud),
    m_index_image( t_index_image)
  {
    const int num_rows = t_index_image.size();
    const int num_cols = t_index_image[0].size();
    m_integ_matrix.resize( num_rows );
    for ( auto & v : m_integ_matrix){
      v.resize( num_cols);
    }
  };

  IntegralCell IntegralImage::fetchPointSum(const int r,const int c){
    list<DataPoint> list = m_index_image[r][c];
    IntegralCell accumulator = IntegralCell();
    if ( list.size()>0){ 
      for( auto & elem: list){
        PointNormalColor3f curr_point = m_cloud[elem.getIndexContainer()];
        Eigen::Vector3f p_j = curr_point.coordinates();
        IntegralCell current_cell_elem = IntegralCell(
            p_j,
            p_j* p_j.transpose(),
            1);
        accumulator = accumulator + current_cell_elem;
      }
    }
    return accumulator;
  }


  void IntegralImage::buildIntegMatrix(){
    for( int r = 0; r < m_integ_matrix.size(); ++r){
      for( int c = 0; c < m_integ_matrix[0].size(); ++c){
        IntegralCell j;
        IntegralCell k;
        IntegralCell l;
        IntegralCell x  = fetchPointSum( r, c);
        if( r-1 < 0 and c-1 <0){
          j = IntegralCell();
          k = IntegralCell();
          l = IntegralCell();
        }else if( r-1 < 0){
          j = IntegralCell();
          k = m_integ_matrix[r][c-1];
          l = IntegralCell();
        }else if( c-1 < 0){
          j = IntegralCell();
          k = IntegralCell();
          l = m_integ_matrix[r-1][c];
        }else{
          j = m_integ_matrix[r-1][c-1];
          k = m_integ_matrix[r][c-1];
          l = m_integ_matrix[r-1][c];
        }
        IntegralCell m = k + l - j + x;
        m_integ_matrix[r][c] = m;
      }
    }
  };
}

