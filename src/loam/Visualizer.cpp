#include "Visualizer.hpp"

using namespace std;
using namespace srrg2_core;
using namespace srrg2_qgl_viewport;

namespace Loam{

  void Visualizer::drawingSubrutine(ViewerCanvasPtr canvas, const std::string& filename){

    MessageFileSource src;
    src.open(filename);
    BaseSensorMessagePtr msg;
    while( (msg=src.getMessage()) && ViewerCoreSharedQGL::isRunning()){
      PointCloud2Message* cloud = dynamic_cast<PointCloud2Message*>(msg.get());
      if (cloud) {
        Point3fVectorCloud current_point_cloud;
        cloud->getPointCloud(current_point_cloud);
        canvas->pushColor();
        canvas->setColor( Vector3f(0.f,0.f,0.f));
        canvas->pushPointSize();
        canvas->setPointSize(1.0);
        canvas->putPoints( current_point_cloud );
        canvas->popAttribute();
        canvas->popAttribute();
        canvas->flush();
      }
    }
  }

  void Visualizer::drawingAxes(ViewerCanvasPtr canvas ){

    while(ViewerCoreSharedQGL::isRunning()){
      int num_points = 100;
      Point3fVectorCloud pointcloud_x_axis;
      Point3fVectorCloud pointcloud_y_axis;
      Point3fVectorCloud pointcloud_z_axis;
      pointcloud_x_axis.resize( num_points);
      pointcloud_y_axis.resize( num_points);
      pointcloud_z_axis.resize( num_points);
      float x = 0;
      float y = 0;
      float z = 0;
	    
      for (unsigned int i = 0; i < pointcloud_x_axis.size(); ++i) {
        x += 0.1;
        y += 0.1;
        z += 0.1;
        pointcloud_x_axis[i].coordinates()=Vector3f( x,0,0);
        pointcloud_y_axis[i].coordinates()=Vector3f( 0,y,0);
        pointcloud_z_axis[i].coordinates()=Vector3f( 0,0,z);
      }

      canvas->pushPointSize();
      canvas->setPointSize(4.0);

      canvas->pushColor();
      canvas->setColor(ColorPalette::color3fOrange());
      canvas->putPoints( pointcloud_x_axis);
      canvas->popAttribute();

      canvas->pushColor();
      canvas->setColor(ColorPalette::color3fBlue());
      canvas->putPoints( pointcloud_y_axis);
      canvas->popAttribute();

      canvas->pushColor();
      canvas->setColor(ColorPalette::color3fGreen());
      canvas->putPoints( pointcloud_z_axis);
      canvas->popAttribute();

      canvas->popAttribute();
      canvas->flush();
    }
  }

  void Visualizer::visualizeCondition( ViewerCanvasPtr canvas){

    canvas->flush();
    int num_points_axes = 100;
    Point3fVectorCloud pointcloud_x_axis;
    Point3fVectorCloud pointcloud_y_axis;
    Point3fVectorCloud pointcloud_z_axis;
    pointcloud_x_axis.resize( num_points_axes);
    pointcloud_y_axis.resize( num_points_axes);
    pointcloud_z_axis.resize( num_points_axes);
    float x = 0;float y = 0;float z = 0;
    for (unsigned int i = 0; i < pointcloud_x_axis.size(); ++i) {
      x += 0.1;
      y += 0.1;
      z += 0.1;
      pointcloud_x_axis[i].coordinates()=Vector3f( x,0,0);
      pointcloud_y_axis[i].coordinates()=Vector3f( 0,y,0);
      pointcloud_z_axis[i].coordinates()=Vector3f( 0,0,z);
    }

    float j = 0;float k = 0; const float l = 0;
    float radius = 1.;
    vector<Vector3f> circle_points;
    circle_points.reserve(360);
    for( float angle = 0; angle <= 2*M_PI; angle+= M_PI/180){
      j = radius* cos( angle); 
      k = radius* sin( angle); 
      circle_points.push_back( Vector3f( j, k, l));
    }
    Point3fVectorCloud circle_point_cloud;
    circle_point_cloud.resize( circle_points.size());
    for (unsigned int i = 0; i < circle_point_cloud.size(); ++i) {
      circle_point_cloud[i].coordinates() = circle_points[i];
    }
    canvas->pushPointSize();
    canvas->setPointSize(2.0);
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
            Point3fVectorCloud point_vec;
            point_vec.resize(1);
            point_vec[0].coordinates() = circle_point_cloud[j].coordinates();
            canvas->pushColor();
            canvas->setColor( curr_color);
            canvas->putPoints(point_vec);
            canvas->popAttribute();
          }
        }
        canvas->popAttribute();
        canvas->pushPointSize();
        canvas->setPointSize(3.0);
        canvas->pushColor();
        canvas->setColor(ColorPalette::color3fOrange());
        canvas->putPoints( pointcloud_x_axis);
        canvas->popAttribute();
        canvas->pushColor();
        canvas->setColor(ColorPalette::color3fGreen());
        canvas->putPoints( pointcloud_y_axis);
        canvas->popAttribute();
        canvas->pushColor();
        canvas->setColor(ColorPalette::color3fBlue());
        canvas->putPoints( pointcloud_z_axis);
        canvas->popAttribute();
        canvas->popAttribute();
        canvas->flush();
      }
    }
  }



  void Visualizer::visualizeSphere( ViewerCanvasPtr canvas, const  string & filename){

    canvas->flush();

    int num_points_axes = 100;
    Point3fVectorCloud pointcloud_x_axis;
    Point3fVectorCloud pointcloud_y_axis;
    Point3fVectorCloud pointcloud_z_axis;
    pointcloud_x_axis.resize( num_points_axes);
    pointcloud_y_axis.resize( num_points_axes);
    pointcloud_z_axis.resize( num_points_axes);
    float x = 0;float y = 0;float z = 0;
    for (unsigned int i = 0; i < pointcloud_x_axis.size(); ++i) {
      x += 0.1;
      y += 0.1;
      z += 0.1;
      pointcloud_x_axis[i].coordinates()=Vector3f( x,0,0);
      pointcloud_y_axis[i].coordinates()=Vector3f( 0,y,0);
      pointcloud_z_axis[i].coordinates()=Vector3f( 0,0,z);
    }

    messages_registerTypes();
    MessageFileSource src;
    src.open(filename);
    BaseSensorMessagePtr msg;
    canvas->pushPointSize();
    canvas->setPointSize(1.0);
    float normalized_radius = 5.;
    msg=src.getMessage();
    if ( msg){
      PointCloud2Message* cloud = dynamic_cast<PointCloud2Message*>(msg.get());
      Point3fVectorCloud point_cloud;
      cloud->getPointCloud(point_cloud);
      Point3fVectorCloud sphere_cloud;
      sphere_cloud.resize( point_cloud.size());
      for( int i = 0; i<point_cloud.size(); ++i){
        Point3f p = point_cloud[i];
        Vector3f spherical_coords = SphericalDepthImage::directMappingFunc( p.coordinates());
        spherical_coords.z() = normalized_radius;
        sphere_cloud[i].coordinates() = SphericalDepthImage::inverseMappingFunc( spherical_coords);
      }
       
      while(  ViewerCoreSharedQGL::isRunning()){
        canvas->pushColor();
        canvas->setColor(Vector3f(0,0,0));
        canvas->putPoints(sphere_cloud);
        canvas->pushColor();
        canvas->setColor(ColorPalette::color3fOrange());
        canvas->putPoints( pointcloud_x_axis);
        canvas->popAttribute();
        canvas->pushColor();
        canvas->setColor(ColorPalette::color3fGreen());
        canvas->putPoints( pointcloud_y_axis);
        canvas->popAttribute();
        canvas->pushColor();
        canvas->setColor(ColorPalette::color3fBlue());
        canvas->putPoints( pointcloud_z_axis);
        canvas->popAttribute();
        canvas->flush();
      }
    }
  }

  void Visualizer::visualizeCloud( ViewerCanvasPtr canvas, const  string & filename){

    canvas->flush();
    int num_points_axes = 100;
    Point3fVectorCloud pointcloud_x_axis;
    Point3fVectorCloud pointcloud_y_axis;
    Point3fVectorCloud pointcloud_z_axis;
    pointcloud_x_axis.resize( num_points_axes);
    pointcloud_y_axis.resize( num_points_axes);
    pointcloud_z_axis.resize( num_points_axes);
    float x = 0;float y = 0;float z = 0;
    for (unsigned int i = 0; i < pointcloud_x_axis.size(); ++i) {
      x += 0.1;
      y += 0.1;
      z += 0.1;
      pointcloud_x_axis[i].coordinates()=Vector3f( x,0,0);
      pointcloud_y_axis[i].coordinates()=Vector3f( 0,y,0);
      pointcloud_z_axis[i].coordinates()=Vector3f( 0,0,z);
    }
    messages_registerTypes();
    MessageFileSource src;
    src.open(filename);
    BaseSensorMessagePtr msg;

    float c;
    const Vector3f base_color = Vector3f( 0.4,0.4,0.4);
    while( (msg=src.getMessage()) && ViewerCoreSharedQGL::isRunning()){
      PointCloud2Message* cloud = dynamic_cast<PointCloud2Message*>(msg.get());
      if (cloud) {
        canvas->pushPointSize();
        canvas->setPointSize(1.0);

        Point3fVectorCloud current_point_cloud;
        cloud->getPointCloud(current_point_cloud);
        for (unsigned int i = 0; i < current_point_cloud.size(); ++i) {
          cerr<<"point number :"<<i<<"\n";
          c = FeatureExtractor::computeSmoothness( current_point_cloud, i);
          Vector3f curr_color = base_color * c;
          Point3fVectorCloud point_vec;
          point_vec.resize(1);
          point_vec[0].coordinates() = current_point_cloud[i].coordinates();
          canvas->pushColor();
          canvas->setColor( curr_color);
          canvas->putPoints(point_vec);
          canvas->popAttribute();
        }
        canvas->pushPointSize();
        canvas->setPointSize(3.0);
        canvas->pushColor();
        canvas->setColor(ColorPalette::color3fOrange());
        canvas->putPoints( pointcloud_x_axis);
        canvas->popAttribute();
        canvas->pushColor();
        canvas->setColor(ColorPalette::color3fGreen());
        canvas->putPoints( pointcloud_y_axis);
        canvas->popAttribute();
        canvas->pushColor();
        canvas->setColor(ColorPalette::color3fBlue());
        canvas->putPoints( pointcloud_z_axis);
        canvas->popAttribute();
        canvas->popAttribute();
        canvas->flush();
      }
    }
  }

  void Visualizer::visualizeFullClouds( ViewerCanvasPtr canvas, const  string & filename){

    canvas->flush();
    int num_points_axes = 100;
    Point3fVectorCloud pointcloud_x_axis;
    Point3fVectorCloud pointcloud_y_axis;
    Point3fVectorCloud pointcloud_z_axis;
    pointcloud_x_axis.resize( num_points_axes);
    pointcloud_y_axis.resize( num_points_axes);
    pointcloud_z_axis.resize( num_points_axes);
    float x = 0;float y = 0;float z = 0;
    for (unsigned int i = 0; i < pointcloud_x_axis.size(); ++i) {
      x += 0.1;
      y += 0.1;
      z += 0.1;
      pointcloud_x_axis[i].coordinates()=Vector3f( x,0,0);
      pointcloud_y_axis[i].coordinates()=Vector3f( 0,y,0);
      pointcloud_z_axis[i].coordinates()=Vector3f( 0,0,z);
    }
    messages_registerTypes();
    MessageFileSource src;
    src.open(filename);
    BaseSensorMessagePtr msg;

    canvas->pushPointSize();
    canvas->setPointSize(1.0);
    while( (msg=src.getMessage()) && ViewerCoreSharedQGL::isRunning()){
      PointCloud2Message* cloud = dynamic_cast<PointCloud2Message*>(msg.get());
      if (cloud) {
        Point3fVectorCloud current_point_cloud;
        cloud->getPointCloud(current_point_cloud);
        canvas->pushColor();
        canvas->setColor(Vector3f(0,0,0));
        canvas->putPoints(current_point_cloud);
        canvas->pushColor();
        canvas->setColor(ColorPalette::color3fOrange());
        canvas->putPoints( pointcloud_x_axis);
        canvas->popAttribute();
        canvas->pushColor();
        canvas->setColor(ColorPalette::color3fGreen());
        canvas->putPoints( pointcloud_y_axis);
        canvas->popAttribute();
        canvas->pushColor();
        canvas->setColor(ColorPalette::color3fBlue());
        canvas->putPoints( pointcloud_z_axis);
        canvas->popAttribute();
        canvas->flush();
      }
    }
  }

  void Visualizer::visualizeCleanedClouds( ViewerCanvasPtr first_canvas, ViewerCanvasPtr second_canvas, const  string & filename){
    const int num_rings =60;
    const int num_points_ring= 2000;
    const float epsilon_radius= 0.2;
    const int epsilon_times= 10;
    SphericalDepthImage sph_Image;
    Point3fVectorCloud cleanedCloud;
       
    first_canvas->flush();
    second_canvas->flush();

    int num_points_axes = 100;
    Point3fVectorCloud pointcloud_x_axis;
    Point3fVectorCloud pointcloud_y_axis;
    Point3fVectorCloud pointcloud_z_axis;
    pointcloud_x_axis.resize( num_points_axes);
    pointcloud_y_axis.resize( num_points_axes);
    pointcloud_z_axis.resize( num_points_axes);
    float x = 0;float y = 0;float z = 0;
    for (unsigned int i = 0; i < pointcloud_x_axis.size(); ++i) {
      x += 0.1;
      y += 0.1;
      z += 0.1;
      pointcloud_x_axis[i].coordinates()=Vector3f( x,0,0);
      pointcloud_y_axis[i].coordinates()=Vector3f( 0,y,0);
      pointcloud_z_axis[i].coordinates()=Vector3f( 0,0,z);
    }
    messages_registerTypes();
    MessageFileSource src;
    src.open(filename);
    BaseSensorMessagePtr msg;

    first_canvas->pushPointSize();
    first_canvas->setPointSize(1.0);
    second_canvas->pushPointSize();
    second_canvas->setPointSize(1.0);
    
    while( (msg=src.getMessage()) && ViewerCoreSharedQGL::isRunning()){
      PointCloud2Message* cloud = dynamic_cast<PointCloud2Message*>(msg.get());
      if (cloud) {
        Point3fVectorCloud current_point_cloud;
        cloud->getPointCloud(current_point_cloud);
  
        sph_Image= SphericalDepthImage(
            num_rings,num_points_ring,
            epsilon_radius, epsilon_times,
            current_point_cloud);
        sph_Image.removeFlatSurfaces();
        cleanedCloud = sph_Image.getPointCloud();

        first_canvas->pushColor();
        first_canvas->setColor(Vector3f(0,0,0));
        first_canvas->putPoints(cleanedCloud);
        first_canvas->pushColor();

        second_canvas->pushColor();
        second_canvas->setColor(Vector3f(0,0,0));
        second_canvas->putPoints(current_point_cloud);
        second_canvas->pushColor();


        first_canvas->setColor(ColorPalette::color3fOrange());
        first_canvas->putPoints( pointcloud_x_axis);
        first_canvas->popAttribute();
        first_canvas->pushColor();
        first_canvas->setColor(ColorPalette::color3fGreen());
        first_canvas->putPoints( pointcloud_y_axis);
        first_canvas->popAttribute();
        first_canvas->pushColor();
        first_canvas->setColor(ColorPalette::color3fBlue());
        first_canvas->putPoints( pointcloud_z_axis);
        first_canvas->popAttribute();
        first_canvas->flush();

        second_canvas->setColor(ColorPalette::color3fOrange());
        second_canvas->putPoints( pointcloud_x_axis);
        second_canvas->popAttribute();
        second_canvas->pushColor();
        second_canvas->setColor(ColorPalette::color3fGreen());
        second_canvas->putPoints( pointcloud_y_axis);
        second_canvas->popAttribute();
        second_canvas->pushColor();
        second_canvas->setColor(ColorPalette::color3fBlue());
        second_canvas->putPoints( pointcloud_z_axis);
        second_canvas->popAttribute();
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

    canvas->flush();
    int num_points_axes = 100;
    Point3fVectorCloud pointcloud_x_axis;
    Point3fVectorCloud pointcloud_y_axis;
    Point3fVectorCloud pointcloud_z_axis;
    pointcloud_x_axis.resize( num_points_axes);
    pointcloud_y_axis.resize( num_points_axes);
    pointcloud_z_axis.resize( num_points_axes);
    float x = 0;float y = 0;float z = 0;
    for (unsigned int i = 0; i < pointcloud_x_axis.size(); ++i) {
      x += 0.1;
      y += 0.1;
      z += 0.1;
      pointcloud_x_axis[i].coordinates()=Vector3f( x,0,0);
      pointcloud_y_axis[i].coordinates()=Vector3f( 0,y,0);
      pointcloud_z_axis[i].coordinates()=Vector3f( 0,0,z);
    }
 
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
    Point3fVectorCloud pointcloud_line;
    pointcloud_line.resize( num_line_points);
    for (unsigned int j = 0; j < pointcloud_line.size(); ++j) {
      pointcloud_line[j].coordinates()=line_points[j].getCoords();
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
      canvas->popAttribute();
      canvas->pushPointSize();
      canvas->setPointSize(3.0);
      //x_axis
      canvas->pushColor();
      canvas->setColor(ColorPalette::color3fOrange());
      canvas->putPoints( pointcloud_x_axis);
      canvas->popAttribute();
      //y_axis
      canvas->pushColor();
      canvas->setColor(ColorPalette::color3fGreen());
      canvas->putPoints( pointcloud_y_axis);
      canvas->popAttribute();
      //z_axis
      canvas->pushColor();
      canvas->setColor(ColorPalette::color3fBlue());
      canvas->putPoints( pointcloud_z_axis);
      canvas->popAttribute();
      //line
      canvas->pushColor();
      canvas->setColor( Vector3f(0.f,0.5f,0.f));
      canvas->putPoints( pointcloud_line);
      canvas->popAttribute();

      canvas->popAttribute();
      canvas->flush();
    }
  }


}
