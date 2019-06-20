#pragma once
#include "../interfaces/IntegralImageInterface.hpp"

using namespace std;
using namespace srrg2_core;
namespace Loam{

  class IntegralImage: IntegralImageInterface{
    protected:
      PointNormalColor3fVectorCloud m_cloud;
      vector<vector<list<DataPoint>>> m_index_image;
      vector<vector<IntegralCell>> m_integ_matrix;

    public:
      IntegralImage(
         const  PointNormalColor3fVectorCloud & t_cloud,
         vector<vector<list<DataPoint>>> t_index_image); 

      void buildIntegMatrix();

      IntegralCell fetchPointSum(const int r,const int c);

      inline vector<vector<IntegralCell>> getIntegMat(){return m_integ_matrix;};
      
      IntegralCell getCellInsideBoundaries(
        const int t_rowMin,const int t_rowMax,
        const int t_colMin,const int t_colMax);
 
  };
}
  

