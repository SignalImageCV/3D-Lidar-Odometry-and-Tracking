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
//taken from visualizer class
//
//
static void drawingSampledSmoothness( ViewerCanvasPtr canvas);
void Visualizer::drawingSampledSmoothness( ViewerCanvasPtr  canvas){
    //todo solve the error given: in feature extractor the smoothness computation
    //encounters a denominator close to zero even if the points are far from the origin
    Eigen::Vector3f center( 7, 7, 7);
    float length_edge = 1;
    int precision = 9;
    std::vector<ScanPoint> cube_points =
      ScanPoint::generateCubeSampledScanPoints(center, length_edge, precision);

    vector<PointNormalColor3fVectorCloud> axes =  Drawer::createAxes();

    //line creation
    const int num_line_points = 50;
    PointNormalColor3fVectorCloud pointcloud_line;
    pointcloud_line.reserve( num_line_points);
    vector<ScanPoint> line_points;
    line_points.reserve(num_line_points);
    const Eigen::Vector3f line_start( 0, 2, -10); 
    Eigen::Vector3f line_curr_point = line_start; 
    const Eigen::Vector3f line_direction(0, 0, 0.4);
    for (unsigned int i = 0; i < num_line_points; ++i) {
      PointNormalColor3f p;
      p.coordinates()  = line_curr_point;
      p.color() = ColorPalette::color3fDarkCyan();
      p.normal() = Vector3f::Zero();
      pointcloud_line.push_back(p);
      line_curr_point+= line_direction;
    }
    std::vector<Vector3f, Eigen::aligned_allocator<Vector3f> >  colors;
    colors.resize(64);
    for(size_t i=0; i < colors.size(); ++i) {
      colors[i]= Vector3f( 2.f*float(i)/64, 2.f*(1.0 - float(i)/64), 0.f);
    }

    for( auto & p: cube_points){
      //std::cerr<<"coords: "<<p.getCoords()<<"\n";
      //std::cerr<<"bef: "<<p.getSmoothness()<<"\n";
      //FeatureExtractor::computeSingleSmoothnessPaper(line_points, p );
      FeatureExtractor::computeSingleSmoothnessMine(line_points, p );
      //std::cerr<<"aft: "<<p.getSmoothness()<<"\n";
    }
    while(ViewerCoreSharedQGL::isRunning()){
      canvas->pushPointSize();
      canvas->setPointSize(7.0);
      float min_smoothness = FeatureExtractor::findMinSmoothnessPoint(cube_points).getSmoothness();
      float max_smoothness = FeatureExtractor::findMaxSmoothnessPoint(cube_points).getSmoothness();
      double dim_interval= static_cast<double>( (max_smoothness - min_smoothness) / colors.size());

      std::list<ScanPoint> ordered_for_smoothness= FeatureExtractor::sortForIncreasingSmoothness(cube_points);
      //cerr<<"Distribution:min="<<min_smoothness<<" max="<<max_smoothness<<" inter="<<dim_interval<<"\n";

      std::vector<ScanPoint> curr_vec;
      int curr_interval_index = 0; 
      for( std::list<ScanPoint>::iterator it=ordered_for_smoothness.begin();
        it != ordered_for_smoothness.end(); ++it){

        int quotient= static_cast<int>( (it->getSmoothness() - min_smoothness)/dim_interval);
        if (  quotient<= curr_interval_index){
          curr_vec.push_back( *it);
        }
        else{
          //cerr<<"num_index= "<<curr_interval_index<<" num_points: "<<curr_vec.size()<<"\n";
          curr_interval_index = quotient;
          if( curr_vec.size() > 0){
            Point3fVectorCloud pointcloud;
            pointcloud.resize( curr_vec.size());

            for (unsigned int j = 0; j < pointcloud.size(); ++j) {
              pointcloud[j].coordinates()=curr_vec[j].getCoords();
            }
            canvas->pushColor();
            canvas->setColor(colors[curr_interval_index]);
            canvas->putPoints( pointcloud);
            canvas->popAttribute();
            curr_vec.clear();
            curr_vec.push_back(*it);
          }
        }
      }
      canvas->pushPointSize();
      canvas->setPointSize(3.0);
      Visualizer::drawAxes( canvas, axes);
      canvas->putPoints( pointcloud_line);
      canvas->flush();
    }
  }

  static void visualizeCloudSmoothness( ViewerCanvasPtr canvas, const  string & filename);

  void Visualizer::visualizeCloudSmoothness( ViewerCanvasPtr canvas, const  string & filename){
    canvas->flush();
    DatasetManager dM( filename);

    float c;
    const Vector3f base_color = Vector3f( 0.4,0.4,0.4);
    
    while( ViewerCoreSharedQGL::isRunning()){

      PointNormalColor3fVectorCloud current_point_cloud= dM.readMessageFromDataset();
      if( current_point_cloud.size()> 0){
        for (unsigned int i = 0; i < current_point_cloud.size(); ++i) {
          c = FeatureExtractor::computeSmoothness( current_point_cloud, i);
          Vector3f curr_color = base_color * c;
          current_point_cloud[i].color() = curr_color;
        }

        canvas->pushPointSize();
        canvas->setPointSize(1.0);
        canvas->putPoints( current_point_cloud );
        canvas->popAttribute();

        canvas->pushPointSize();
        canvas->setPointSize(3.0);
        canvas->flush();
      }
    }
  }



