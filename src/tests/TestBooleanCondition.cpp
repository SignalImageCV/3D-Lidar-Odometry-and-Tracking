#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg_system_utils/shell_colors.h>
#include <thread>
#include "../loam/MyMath.hpp"

using namespace std;
using namespace srrg2_core;
using namespace srrg2_qgl_viewport;
using namespace Loam;

void visualizeCondition( ViewerCanvasPtr canvas){

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



//bug: the bool condition is never verified


int main( int argc, char** argv){

  ViewerCoreSharedQGL viewer(argc, argv);
  ViewerCanvasPtr canvas = viewer.getCanvas("drawingCloud");
  std::thread processing_thread(visualizeCondition, canvas);
  viewer.startViewerServer();
  processing_thread.join();
  return 0;
}




