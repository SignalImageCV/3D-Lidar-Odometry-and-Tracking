#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg_messages/instances.h>
#include "loam/MyMath.hpp"
#include "loam/DatasetManager.hpp"


using namespace srrg2_core;
using namespace srrg2_core_ros;
using namespace srrg2_qgl_viewport;
using namespace Loam;

const char* banner[] = {
    "dynamic executor",
      0
};

void visualizeSphere(ViewerCanvasPtr canvas,const string & filename);

int main( int argc, char** argv){
  ParseCommandLine cmd_line(argv,banner);
  ArgumentString  dataset (&cmd_line, "d", "dataset", "path to dataset" , "");
  cmd_line.parse();

  messages_registerTypes();
  srrgInit( argc, argv, "hi");
 
  QApplication qapp(argc, argv);
  ViewerCoreSharedQGL viewer(argc, argv, &qapp);
  ViewerCanvasPtr canvas = viewer.getCanvas("drawingSphere");
  std::thread processing_thread(visualizeSphere, canvas, dataset.value());
  viewer.startViewerServer();
  processing_thread.join();
  return 0;
}
  void visualizeSphere( ViewerCanvasPtr canvas, const  string & filename){
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
 



