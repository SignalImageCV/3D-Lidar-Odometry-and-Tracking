#include "Visualizer.hpp"

using namespace std;
using namespace srrg2_core;
using namespace srrg2_qgl_viewport;

namespace Loam{

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



}
