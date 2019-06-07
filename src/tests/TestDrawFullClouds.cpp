#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg_system_utils/shell_colors.h>
#include <thread>
#include "../loam/features/FeatureExtractor.hpp"

using namespace std;
using namespace srrg2_core;
using namespace srrg2_qgl_viewport;
using namespace Loam;


void visualizeFullClouds( ViewerCanvasPtr canvas, const  string & filename){

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





int main( int argc, char** argv){

  string filename = "/home/dinies/temp/trial/tuttty.boss";
  ViewerCoreSharedQGL viewer(argc, argv);
  ViewerCanvasPtr canvas = viewer.getCanvas("drawingCloud");
  std::thread processing_thread(visualizeFullClouds, canvas, filename);
  viewer.startViewerServer();
  processing_thread.join();
  return 0;
}




