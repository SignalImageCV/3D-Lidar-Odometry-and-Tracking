#include "Visualizer.hpp"

using namespace std;
using namespace srrg2_core;
using namespace srrg2_qgl_viewport;

namespace Loam{

  vector<PointNormalColor3fVectorCloud> Visualizer::createAxes(){

    vector< PointNormalColor3fVectorCloud> axes;
    
    int num_points = 100;
    PointNormalColor3fVectorCloud pointcloud_x_axis;
    PointNormalColor3fVectorCloud pointcloud_y_axis;
    PointNormalColor3fVectorCloud pointcloud_z_axis;
    pointcloud_x_axis.resize( num_points);
    pointcloud_y_axis.resize( num_points);
    pointcloud_z_axis.resize( num_points);
    float x = 0;
    float y = 0;
    float z = 0;
    for (unsigned int i = 0; i < num_points; ++i) {
      x += 0.1;
      y += 0.1;
      z += 0.1;
      pointcloud_x_axis[i].coordinates()=Vector3f( x,0,0);
      pointcloud_x_axis[i].color()= ColorPalette::color3fDarkCyan();
      pointcloud_y_axis[i].coordinates()=Vector3f( 0,y,0);
      pointcloud_y_axis[i].color()= ColorPalette::color3fDarkGreen();
      pointcloud_z_axis[i].coordinates()=Vector3f( 0,0,z);
      pointcloud_z_axis[i].color()= ColorPalette::color3fDarkRed();
    }
    axes.push_back(pointcloud_x_axis);
    axes.push_back(pointcloud_y_axis);
    axes.push_back(pointcloud_z_axis);
    return axes;
  }

  PointNormalColor3fVectorCloud Visualizer::createCircle(const float radius = 1.){
    float j = 0;float k = 0; const float l = 0;
    vector<Vector3f> circle_points;
    circle_points.reserve(360);
    for( float angle = 0; angle <= 2*M_PI; angle+= M_PI/180){
      j = radius* cos( angle); 
      k = radius* sin( angle); 
      circle_points.push_back( Vector3f( j, k, l));
    }
    PointNormalColor3fVectorCloud circle_point_cloud;
    circle_point_cloud.resize( circle_points.size());
    for (unsigned int i = 0; i < circle_point_cloud.size(); ++i) {
      circle_point_cloud[i].coordinates() = circle_points[i];
      circle_point_cloud[i].color() = ColorPalette::color3fBlack();
    }
    return circle_point_cloud;
  }

  void Visualizer::drawAxes(ViewerCanvasPtr canvas, const vector<PointNormalColor3fVectorCloud> & t_axes){
    for( auto & axis: t_axes){
      canvas->putPoints( axis);
    }
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

    vector<PointNormalColor3fVectorCloud> axes =  Visualizer::createAxes();
    while(ViewerCoreSharedQGL::isRunning()){
      canvas->pushPointSize();
      canvas->setPointSize(4.0);
      Visualizer::drawAxes( canvas, axes);
      canvas->flush();
    }
  }

  void Visualizer::visualizeCondition( ViewerCanvasPtr canvas){

    vector<PointNormalColor3fVectorCloud> axes =  Visualizer::createAxes();
    PointNormalColor3fVectorCloud circle_point_cloud  = Visualizer::createCircle();

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

        Visualizer::drawAxes( canvas, axes);
        canvas->flush();
      }
    }
  }

  void Visualizer::visualizeSphere( ViewerCanvasPtr canvas, const  string & filename){
    DatasetManager dM( filename);
    vector<PointNormalColor3fVectorCloud> axes =  Visualizer::createAxes();

    float normalized_radius = 5.;
    PointNormalColor3fVectorCloud point_cloud = dM.readMessageFromDataset();

    PointNormalColor3fVectorCloud sphere_cloud;
    sphere_cloud.resize( point_cloud.size());
    for(unsigned int i = 0; i<point_cloud.size(); ++i){
      PointNormalColor3f p = point_cloud[i];
      Vector3f spherical_coords = SphericalDepthImage::directMappingFunc( p.coordinates());
      spherical_coords.z() = normalized_radius;
      sphere_cloud[i].coordinates() = SphericalDepthImage::inverseMappingFunc( spherical_coords);
      sphere_cloud[i].color() = p.color();
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
    vector<PointNormalColor3fVectorCloud> axes =  Visualizer::createAxes();
    float c;
    const Vector3f base_color = Vector3f( 0.4,0.4,0.4);
    
    while( ViewerCoreSharedQGL::isRunning()){

      PointNormalColor3fVectorCloud current_point_cloud= dM.readMessageFromDataset();
      if( current_point_cloud.size()> 0){
        for (unsigned int i = 0; i < current_point_cloud.size(); ++i) {
          cerr<<"point number :"<<i<<"\n";
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
        Visualizer::drawAxes( canvas, axes);
        canvas->flush();
      }
    }
  }

  void Visualizer::visualizeFullClouds( ViewerCanvasPtr canvas, const  string & filename){

    DatasetManager dM( filename);
    vector<PointNormalColor3fVectorCloud> axes =  Visualizer::createAxes();

    while( ViewerCoreSharedQGL::isRunning()){
      PointNormalColor3fVectorCloud current_point_cloud= dM.readMessageFromDataset();
      if( current_point_cloud.size()> 0){
        canvas->putPoints(current_point_cloud);
        Visualizer::drawAxes(canvas, axes);
        canvas->flush();
      }
    }
  }

  void Visualizer::visualizeCleanedClouds( ViewerCanvasPtr first_canvas, ViewerCanvasPtr second_canvas, const  string & filename){
    const sphericalImage_params params(
      60, //num_vertical_rings
      2000, //num_points_ring
      10, //epsilon_times
      0.15, //epsilon_radius
      1, //depth_differential_threshold
      2  //min_neighboors_for_normal
    );
        
    SphericalDepthImage sph_Image;
    DatasetManager dM( filename);

    vector<PointNormalColor3fVectorCloud> axes =  Visualizer::createAxes();

    while( ViewerCoreSharedQGL::isRunning()){

      PointNormalColor3fVectorCloud current_point_cloud= dM.readMessageFromDataset();
      if( current_point_cloud.size()> 0){

 
        sph_Image= SphericalDepthImage(current_point_cloud,params);
        sph_Image.removeFlatSurfaces();

        PointNormalColor3fVectorCloud cleanedCloud= sph_Image.getPointCloud();

        first_canvas->putPoints(cleanedCloud);

        second_canvas->putPoints(current_point_cloud);

        first_canvas->pushPointSize();
        first_canvas->setPointSize(3.0);
        second_canvas->pushPointSize();
        second_canvas->setPointSize(3.0);
        Visualizer::drawAxes( first_canvas, axes);
        Visualizer::drawAxes( second_canvas, axes);

        first_canvas->flush();
        second_canvas->flush();
      }
    }
  }


  void Visualizer::drawingSampledSmoothness( ViewerCanvasPtr  canvas){
    Eigen::Vector3f center( 0, 2, 0);
    float length_edge = 1;
    int precision = 9;
    std::vector<ScanPoint> cube_points =
      ScanPoint::generateCubeSampledScanPoints(center, length_edge, precision);

    vector<PointNormalColor3fVectorCloud> axes =  Visualizer::createAxes();

    //line creation
    const int num_line_points = 50;
    vector<ScanPoint> line_points;
    line_points.reserve(num_line_points);
    const Eigen::Vector3f line_start( 0, 2, -10); 
    Eigen::Vector3f line_curr_point = line_start; 
    const Eigen::Vector3f line_direction(0, 0, 0.4);
    for (unsigned int i = 0; i < num_line_points; ++i) {
      ScanPoint p( 0, i, line_curr_point);
      line_points.push_back( p);
      line_curr_point+= line_direction;
    }
    PointNormalColor3fVectorCloud pointcloud_line;
    pointcloud_line.resize( num_line_points);
    for (unsigned int j = 0; j < pointcloud_line.size(); ++j) {
      pointcloud_line[j].coordinates()=line_points[j].getCoords();
      pointcloud_line[j].color()=ColorPalette::color3fDarkCyan();
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
