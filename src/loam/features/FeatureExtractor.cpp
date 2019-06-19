#include "FeatureExtractor.hpp"

namespace Loam{

  FeatureExtractor::FeatureExtractor( PointNormalColor3fVectorCloud & t_cloud ):
    FeatureExtractorInterface( ),
    m_cloud( t_cloud) {};




  vector<Matchable> FeatureExtractor::extractFeatures(){
    vector<Matchable> v;
    return v;
  }



  float FeatureExtractor::computeSmoothness( const PointNormalColor3fVectorCloud & cloud, const int index_point){
    PointNormalColor3f curr_p = cloud[index_point];
    float range_scan_section_rad = M_PI/6;

    const float theta = atan2( curr_p.coordinates().y(), curr_p.coordinates().x());
    const float alpha =  MyMath::boxMinusAngleRad( theta, range_scan_section_rad/2);
    const float beta =  MyMath::boxPlusAngleRad( theta, range_scan_section_rad/2);
    int dim_S= 0;

    Vector3f sum_distances = Vector3f::Zero();
    for(unsigned int i = 0; i< cloud.size();++i){
      if ( i != index_point){
        float phi = atan2( cloud[i].coordinates().y(), cloud[i].coordinates().x());

        if ( MyMath::checkIsInsideArcBoundaries( alpha, beta, theta, phi)){
          sum_distances += ( curr_p.coordinates() - cloud[i].coordinates() );
          ++dim_S;
        }
      }
    }
    float denom = dim_S * curr_p.coordinates().norm();
    float sigma = 0.001;
    if (denom < sigma){
      denom = sigma;
      std::cerr<<"This is a point near the camera frame origin:\n";
      std::cerr<<" coords: "<<curr_p.coordinates()<<"\n";
      std::cerr<<" dim S set: "<<dim_S <<"\n";
    }
    float c = sum_distances.norm() / denom ;
    return c;
  }

  void FeatureExtractor::computeSmoothnessPaper( std::vector<ScanPoint> & points){
    for(unsigned int i = 0; i<points.size();++i){
      Vector3f sum_distances = Vector3f::Zero();
      for(unsigned int j = 0; j<points.size();++j){
        if ( i != j){
          sum_distances += points[i].getCoords() - points[j].getCoords();
        }
      }
      float denom = points.size() * points[i].getCoords().norm();
      float sigma = 0.001;
      if (denom < sigma){
        denom = sigma;
        std::cerr<<"There is a point near the camera frame origin:\n";
        std::cerr<<" coords: "<<points[i].getCoords()<<"\n";
        std::cerr<<" ofSweep: "<<points[i].getIndexOfSweep()<<"\n";
        std::cerr<<" inSweep: "<<points[i].getIndexInSweep()<<"\n";
      }
      float c = sum_distances.norm() / denom ;
      points[i].setSmoothness( c );
    }
  };

  void FeatureExtractor::computeSmoothnessMine( std::vector<ScanPoint> & points){
    for(unsigned int i = 0; i<points.size();++i){
      Vector3f sum_distances = Vector3f::Zero();
      for(unsigned int j = 0; j<points.size();++j){
        if ( i != j){
          sum_distances += points[i].getCoords() - points[j].getCoords();
        }
      }
      float denom = points.size();
      //float denom = points.size() * points[i].getCoords().norm();
      float sigma = 0.001;
      if (denom < sigma){
        denom = sigma;
        std::cerr<<"There is a point near the camera frame origin:\n";
        std::cerr<<" coords: "<<points[i].getCoords()<<"\n";
        std::cerr<<" ofSweep: "<<points[i].getIndexOfSweep()<<"\n";
        std::cerr<<" inSweep: "<<points[i].getIndexInSweep()<<"\n";
      }
      float c = sum_distances.norm() / denom ;
      points[i].setSmoothness( c);
    }
  };

  void FeatureExtractor::computeSingleSmoothnessPaper(const std::vector<ScanPoint> & other_points, ScanPoint & point){
    Vector3f sum_distances = Vector3f::Zero();
    for(auto&  p: other_points){
      sum_distances += point.getCoords() - p.getCoords();
    }
    float denom = other_points.size() * point.getCoords().norm();
    float sigma = 0.001;
    if (denom < sigma){
      denom = sigma;
      std::cerr<<"There is a point near the camera frame origin:\n";
      std::cerr<<" coords: "<<point.getCoords()<<"\n";
      std::cerr<<" ofSweep: "<<point.getIndexOfSweep()<<"\n";
      std::cerr<<" inSweep: "<<point.getIndexInSweep()<<"\n";
    }
    float c = sum_distances.norm() / denom ;
    point.setSmoothness( c);
  };

  void FeatureExtractor::computeSingleSmoothnessMine(const  std::vector<ScanPoint> & other_points, ScanPoint & point){
    Vector3f sum_distances = Vector3f::Zero();
    for(auto&  p: other_points){
      sum_distances += point.getCoords() - p.getCoords();
    }
    float denom = other_points.size();
    //float denom = other_points.size() * point.getCoords().norm();
    float sigma = 0.001;
    if (denom < sigma){
      denom = sigma;
      std::cerr<<"There is a point near the camera frame origin:\n";
      std::cerr<<" coords: "<<point.getCoords()<<"\n";
      std::cerr<<" ofSweep: "<<point.getIndexOfSweep()<<"\n";
      std::cerr<<" inSweep: "<<point.getIndexInSweep()<<"\n";
    }
    float c = sum_distances.norm() / denom ;
    point.setSmoothness( c);
  };



     
  ScanPoint FeatureExtractor::findMaxSmoothnessPoint( const  std::vector<ScanPoint> & points){
    float curr_max_value = points[0].getSmoothness();
    ScanPoint curr_max_point = points[0];

    for( auto& p: points){
      if( p.getSmoothness() > curr_max_value){
        curr_max_point = p;
        curr_max_value = p.getSmoothness();
      }
    }
    return curr_max_point;
  };

  ScanPoint FeatureExtractor::findMinSmoothnessPoint( const  std::vector<ScanPoint> & points){
    float curr_min_value = points[0].getSmoothness();
    ScanPoint curr_min_point = points[0];


    for( auto& p: points){
      if( p.getSmoothness() < curr_min_value){
        curr_min_point = p;
        curr_min_value = p.getSmoothness();
      }
    }
    return curr_min_point;
  }

  std::list<ScanPoint> FeatureExtractor::sortForDecreasingSmoothness(const  std::vector<ScanPoint> & points){
    std::list<ScanPoint> point_list( points.begin(), points.end()); 
    point_list.sort([](const ScanPoint & sp1, const ScanPoint & sp2)
        {
          return sp1.getSmoothness() > sp2.getSmoothness();
        });
    return point_list;
  }
 
  std::list<ScanPoint> FeatureExtractor::sortForIncreasingSmoothness(const  std::vector<ScanPoint> & points){
    std::list<ScanPoint> point_list( points.begin(), points.end()); 
    point_list.sort([](const ScanPoint & sp1, const ScanPoint & sp2)
        {
          return sp1.getSmoothness() < sp2.getSmoothness();
        });
    return point_list;
  }

  std::vector<std::vector<ScanPoint>> FeatureExtractor::divideInSectors( const int num_sectors, const  std::vector<ScanPoint> & points){
    std::vector<std::vector<ScanPoint>> divided_points;
    if( points.size() >= num_sectors){
      divided_points.reserve(num_sectors);
      std::div_t division;
      division= std::div( points.size(), num_sectors);
      int num_point_each_sector= division.quot;
      int remainder= division.rem;
      int current_index = 0;
      int curr_sector_capacity;
      for( int sect = 0; sect<num_sectors; ++sect){
        std::vector<ScanPoint> sector_points;
        sector_points.reserve( num_point_each_sector);
        if( sect < num_sectors - 1){
          curr_sector_capacity = num_point_each_sector;
        }else{
          curr_sector_capacity = num_point_each_sector+remainder;
        }
        for( int j=0; j<curr_sector_capacity; ++j){
          sector_points.push_back( points[current_index + j]);
        }
        divided_points.push_back( sector_points);
        current_index += num_point_each_sector;
      }
    }
    return divided_points;
  };

}




