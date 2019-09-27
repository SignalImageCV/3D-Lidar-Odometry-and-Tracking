#include "ScanPoint.hpp"

namespace Loam {
  std::vector<ScanPoint> ScanPoint::generateCubeSampledScanPoints
    ( const  Eigen::Vector3f & cube_center,
      const float & edge_length,
      const int  precision){

    const float interval = edge_length/ precision;
    std::vector<ScanPoint> sampled_points; 
    sampled_points.reserve( pow( (precision +1), 3));
    const int index_ofSweep = -1;
    int index_inSweep = 0;
    for( float z = cube_center(2)-(edge_length/2);
        z<=cube_center(2)+(edge_length/2); 
        z+=interval){

      for( float y = cube_center(1)-(edge_length/2);
        y<=cube_center(1)+(edge_length/2); 
        y+=interval){

        for( float x = cube_center(0)-(edge_length/2);
          x<=cube_center(0)+(edge_length/2); 
          x+=interval){
            ScanPoint p( index_ofSweep, index_inSweep, x, y, z);
            sampled_points.push_back( p);
            ++index_inSweep;
        }
      }
    }
    return sampled_points;
  }
}
