#include <srrg_system_utils/system_utils.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg_messages/instances.h>
#include "loam/Drawer.hpp"



using namespace srrg2_core;
using namespace srrg2_qgl_viewport;
using namespace Loam;


void visualizeCondition( ViewerCanvasPtr canvas);

int main( int argc, char** argv){

  QApplication qapp(argc, argv);
  ViewerCoreSharedQGL viewer(argc, argv, &qapp);
  ViewerCanvasPtr canvas = viewer.getCanvas("drawingCloud");
  std::thread processing_thread(visualizeCondition, canvas);
  viewer.startViewerServer();
  processing_thread.join();
  return 0;
}

void visualizeCondition( ViewerCanvasPtr canvas){

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



