#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg_system_utils/shell_colors.h>
#include <thread>
#include "../loam/DatasetManager.hpp"
#include "../loam/features/FeatureExtractor.hpp"

using namespace std;
using namespace srrg2_core;
using namespace srrg2_qgl_viewport;
using namespace Loam;

void drawingSmoothness( ViewerCanvasPtr canvas, const  string & filename){


  DatasetManager dM( filename);
  vector<ScanPoint> points;
  FeatureExtractor fE(0);

  while(ViewerCoreSharedQGL::isRunning()){

    points = dM.readMessageFromDataset();
    if(points.size() > 0){
      fE.computeSmoothness( points);
    }




    //Delete the final part, is of another test |||

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


int main( int argc, char** argv){

  string filename = "/home/dinies/gitrepos/loam/datasets/undulating_motion.boss";
  ViewerCoreSharedQGL viewer(argc, argv);
  ViewerCanvasPtr canvas = viewer.getCanvas("drawingPoints_test");
  std::thread processing_thread(drawingSmoothness, canvas, filename);
  viewer.startViewerServer();
  processing_thread.join();
  return 0;
}

