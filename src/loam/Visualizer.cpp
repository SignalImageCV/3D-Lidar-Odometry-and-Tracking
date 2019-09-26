#include "Visualizer.hpp"

using namespace std;
using namespace srrg2_core;
using namespace srrg2_qgl_viewport;

namespace Loam{

 

  void Visualizer::drawAxes(ViewerCanvasPtr canvas, const vector<PointNormalColor3fVectorCloud> & t_axes){
    for( auto & axis: t_axes){
      canvas->putPoints( axis);
    }
  }


  void Visualizer::visualizeCloud( ViewerCanvasPtr canvas,const PointNormalColor3fVectorCloud & t_cloud, const float t_points_size){
    while( ViewerCoreSharedQGL::isRunning()){
        canvas->pushPointSize();
        canvas->setPointSize(t_points_size);
        canvas->putPoints( t_cloud);
        canvas->flush();
    }
  }

  void Visualizer::drawNormals(ViewerCanvasPtr canvas, const PointNormalColor3fVectorCloud & t_points){

    std::vector<Vector3f, Eigen::aligned_allocator<Vector3f> >  colors;
    colors.resize(64);
    for(size_t i=0; i < colors.size(); ++i) {
      colors[i]= Vector3f( 2.f*float(i)/64, 2.f*(1.0 - float(i)/64), 0.f);
    }


    const int starting_color_index = 20;
    const int num_normal_points = 10;
    PointNormalColor3fVectorCloud normalPoints;
    normalPoints.reserve( t_points.size() * num_normal_points);
    for( auto & p: t_points){

      Eigen::Vector3f normal_vec = p.normal();
     
      Eigen::Vector3f normal_direction = Vector3f(normal_vec / num_normal_points);

      Eigen::Vector3f curr_normal_p= p.coordinates(); 
      for(int i = 0; i< num_normal_points; ++i){
        curr_normal_p += normal_direction;
        PointNormalColor3f new_point;
        new_point.coordinates() = curr_normal_p;
        new_point.color() = colors[i+ starting_color_index];
        normalPoints.push_back( new_point);
      }
    }
    canvas->putPoints( normalPoints);
 }


  void Visualizer::visualizeSubrutine(ViewerCanvasPtr canvas, const std::string& filename){

    DatasetManager dM( filename);
    while( ViewerCoreSharedQGL::isRunning()){
      PointNormalColor3fVectorCloud point_cloud = dM.readMessageFromDataset();
      if (point_cloud.size()>0) {
        canvas->pushPointSize();
        canvas->setPointSize(.5);
        canvas->putPoints( point_cloud );
        canvas->flush();
      }
    }
  }

  void Visualizer::visualizeAxes(ViewerCanvasPtr canvas ){

    vector<PointNormalColor3fVectorCloud> axes =  Drawer::createAxes();
    while(ViewerCoreSharedQGL::isRunning()){
      canvas->pushPointSize();
      canvas->setPointSize(4.0);
      Visualizer::drawAxes( canvas, axes);
      canvas->flush();
    }
  }


  void Visualizer::visualizeCondition( ViewerCanvasPtr canvas){

    PointNormalColor3fVectorCloud circle_point_cloud  = Drawer::createCircle(1.);

    float range_scan_section = M_PI/3;
    Vector3f curr_color = Vector3f::Zero();
    while( ViewerCoreSharedQGL::isRunning()){
      for (unsigned int i = 0; i < circle_point_cloud.size(); ++i) {
        for (unsigned int j = 0; j < circle_point_cloud.size(); ++j) {
          if (i !=j){
            float theta = atan2( circle_point_cloud[i].coordinates().y(), circle_point_cloud[i].coordinates().x());
            float alpha = MyMath::boxMinusAngleRad( theta, range_scan_section/2);
            float beta =  MyMath::boxPlusAngleRad( theta, range_scan_section/2);
            float phi = atan2( circle_point_cloud[j].coordinates().y(), circle_point_cloud[j].coordinates().x());
            if ( MyMath::checkIsInsideArcBoundaries( alpha, beta, theta, phi)){
              curr_color = Vector3f( 0.9,0.2,0.2);
            }
            else{
              curr_color = Vector3f( 0.2,0.1,0.9);
            }
            circle_point_cloud[j].color() = curr_color;
          }
        }
        canvas->pushPointSize();
        canvas->setPointSize(2.0);
        canvas->putPoints( circle_point_cloud );
        canvas->flush();
      }
    }
  }

  void Visualizer::visualizePointsWithNormals(ViewerCanvasPtr canvas){

    const int num_line_points = 10;
    PointNormalColor3fVectorCloud line_points;
    line_points.reserve(num_line_points);

    const Eigen::Vector3f line_start( 0, 0, 4); 
    const Eigen::Vector3f line_direction(0, 1, 0);

    const Eigen::Vector3f normal(1., 1., 0.);

    Eigen::Vector3f line_curr_point = line_start; 
    
    for (float i = 0; i < num_line_points; ++i) {
      PointNormalColor3f p;
      p.coordinates() = line_curr_point;
      p.color()= ColorPalette::color3fBlack();
      p.normal()= normal;
      line_points.push_back( p);
      line_curr_point+= line_direction;
    }
    while(  ViewerCoreSharedQGL::isRunning()){
      canvas->putPoints(line_points);
      Visualizer::drawNormals(canvas, line_points);
      canvas->flush();
    }
  }

  void Visualizer::visualizeSphere( ViewerCanvasPtr canvas, const  string & filename){
    DatasetManager dM( filename);
    vector<PointNormalColor3fVectorCloud> axes =  Drawer::createAxes();

    float normalized_radius = 5.;
    PointNormalColor3fVectorCloud point_cloud = dM.readMessageFromDataset();

    PointNormalColor3fVectorCloud sphere_cloud;
    sphere_cloud.resize( point_cloud.size());
    for(unsigned int i = 0; i<point_cloud.size(); ++i){
      PointNormalColor3f p = point_cloud[i];
      Vector3f spherical_coords = MyMath::directMappingFunc( p.coordinates());
      spherical_coords.z() = normalized_radius;
      sphere_cloud[i].coordinates() = MyMath::inverseMappingFunc( spherical_coords);
      sphere_cloud[i].color() = p.color();
      sphere_cloud[i].normal() = p.normal();
    }
     
    while(  ViewerCoreSharedQGL::isRunning()){
      canvas->putPoints(sphere_cloud);
      Visualizer::drawAxes( canvas, axes);
      canvas->flush();
    }
  }
 
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
}
